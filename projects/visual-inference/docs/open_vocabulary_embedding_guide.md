# Open-Vocabulary Semantic Embeddings & Edge Vocabulary Guide

This document explains how open-vocabulary semantic classification works in the active **OWLv2-distilled `MobileNetV4 + P2LitePAN`** HBB and Oriented-Quad detectors, how the **37-class full ontology**, **Core-27 deployment preset**, and **7 background distractors** are organized, and how to build custom zero-shot vocabularies offline with **zero detector retraining**.

For the full development pipeline, training ADRs, and benchmarks, see [`owlv2_pipeline_and_decisions.md`](owlv2_pipeline_and_decisions.md).

---

## 1. Architectural Philosophy: Decoupled OWLv2-Aligned Detection

Instead of locking detection heads to fixed logits over `C` closed-set classes, both `StudentDetector` (HBB) and `QuadProposalDetector` (Oriented Quad) predict **class-agnostic proposals** alongside **L2-normalized 768-D visual embeddings** aligned to **OWLv2-Large's** multimodal space (`teacher_space="owlv2"`, zero runtime text encoder).

### Two Co-Trained Semantic Heads ([`../student_detector/open_vocab_heads.py`](../student_detector/open_vocab_heads.py))

1. **Option A — `DenseAnchorSemanticHead` (`0.135M` params, zero RoI ops)**:
   - Runs a shared 2-layer `DepthwiseSeparableConv` tower across `P2LitePAN` pyramid levels `P2`–`P5` plus a `1×1` Conv2d projection to unit L2 `[N, 768]`.
   - Supports bit-exact `forward_dense` (pure Conv2d for static ONNX/TFLite/NPU graphs, `11.39–11.40 GFLOPs` E2E at `384×384`) and sparse `forward_indexed`.
2. **Option B — `RoIPooledSemanticHead` (`2.875M` params, coordinate-free focal+context RoI)**:
   - Pools `5×5` interior (`focal_margin=0.15`) + exterior (`context_margin=0.45`) features from `P2`–`P5` via `MultiScaleRoIAlign`, then projects through `LayerNorm -> Linear -> GELU -> Dropout -> Linear` to unit L2 `[N, 768]` (`8.60 GFLOPs` E2E at `25` proposals).
   - Deliberately **coordinate-free** (no `[xc/W, yc/H]` positional shortcut) to avoid spatial dataset biases.

> Note: Legacy `teacher_space="siglip"` (with `RegionEmbeddingHead` and centroid/median bias shifting) remains loadable for backward compatibility, but active checkpoints train natively in `teacher_space="owlv2"`.

### OWLv2 Sigmoid Scoring & Distractor Veto (`_ContrastiveClassifierMixin`)

Given unit-normalized visual embeddings $Z_{\text{vis}} \in \mathbb{R}^{N \times 768}$, unit-normalized vocabulary embeddings $W_{\text{fg}} \in \mathbb{R}^{C \times 768}$, and unit-normalized distractor embeddings $W_{\text{dist}} \in \mathbb{R}^{7 \times 768}$:

1. **Affine Cosine Logits** (`teacher_space="owlv2"`):
   - Default initialization `init_scale=20.0`, `init_bias=-6.5` (learned `log_scale` clamped to `[5.0, 125.0]`, no centroid subtraction):
     $$\mathbf{s}_{\text{fg}} = Z_{\text{vis}} W_{\text{fg}}^T, \qquad \mathbf{l}_{\text{base}} = \text{scale} \cdot \mathbf{s}_{\text{fg}} + b$$
2. **Background Distractor Margin Veto** (`apply_calibration=True`):
   - If a proposal matches any background distractor closer than a foreground class within margin `0.02`, its logit is penalized (`1.5 * scale`):
     $$\mathbf{l} = \mathbf{l}_{\text{base}} - 1.5 \cdot \text{scale} \cdot \text{ReLU}\!\left(\max_{d} (Z_{\text{vis}} W_{\text{dist}}^T) - \mathbf{s}_{\text{fg}} + 0.02\right)$$
   - When `C > 1`, an additional top-1 competitive margin `0.25 * scale * ReLU(max_c s_fg - s_fg)` sharpens calibration before `p = sigmoid(logits)`.
3. **Joint Score & Post-Semantic Filtering (`filter_semantic_detections`)**:
   - Combined ranking score is $\sqrt{s_{\text{obj}} \cdot p_{\text{cls}}}$.
   - `filter_semantic_detections` combines Quad + HBB overlap (`max(q_iou, 0.85 * b_iou)`), drops `<4.5px` specks unless `p >= 0.45`, runs objectness-aware container suppression honoring `VALID_CONTAINER_PARTS`, and skips backdrop suppression via `BACKDROP_NO_SUPPRESS_CONTAINERS = {"building", "tree", "fence", "canopy"}`.

---

## 2. Canonical Ontology, Core-27 Preset & 7 Distractors

Defined in [`../student_detector/ontology.py`](../student_detector/ontology.py):

| Ontology Subset | Count | Categories | Role |
|---|---:|---|---|
| **`BASE_CATEGORIES`** | 6 | `car`, `pedestrian`, `cyclist`, `traffic_cone`, `bus`, `truck` | Core dynamic road/navigation agents |
| **`NOVEL_CATEGORIES`** | 5 | `stroller`, `dog`, `debris`, `wheelchair`, `construction_barrier` | Rare / safety-critical obstacles |
| **`EXTENDED_CATEGORIES`** | 26 | `pillar`, `trash_bin`, `street_light`, `parking_slot`, `parking_meter`, `fire_hydrant`, `canopy`, `cart`, `golf_cart`, `luggage_and_bags`, `signboard`, `traffic_sign`, `traffic_light`, `pole`, `bench`, `license_plate`, `window`, `door`, `building`, `tree`, `stairs`, `fence`, `chair`, `table`, `cabinet`, `wheeled_transport` | Urban infrastructure, street furniture, architectural & indoor classes (`OPEN_VOCAB_CATEGORIES` = 6 + 5 + 26 = **37**) |
| **`CORE_27_CATEGORIES`** | **27** | `OPEN_VOCAB_CATEGORIES` minus `PRUNED_SCARCE_CATEGORIES` (`9`) & `street_light` (`CLOSED_MERGE_MAP["street_light"] = "pole"`) | Clean deployment preset; removes rare-label noise (`parking_slot`, `stairs`, `debris`, `golf_cart`, `parking_meter`, `canopy`, `wheeled_transport`, `cart`, `wheelchair`) & merges `street_light -> pole` via `VocabularyBank.subset` |
| **`ZERO_SHOT_EVAL_CATEGORIES`** | 4 | `book`, `painting`, `cushion`, `flowerpot` | Held out from closed-set GT supervision to test true zero-shot retention (`31` total with Core-27, `41` with Full-37) |
| **`DISTRACTOR_CATEGORIES`** | 7 | `road_surface`, `sky`, `vegetation`, `building_wall`, `ego_platform_bodywork`, `indoor_lamp_or_light_fixture`, `bed_and_bedding` | Background absorbers attached to `head.set_distractor_embeddings(...)` for sigmoid veto (never rendered as foreground boxes) |

### Frozen vs. Adapted Vocabulary Banks
- **Frozen OWLv2 Banks** (`artifacts/sota_open_vocab/owlv2_vocabulary_bank_*.pt`): Encoded purely by OWLv2-Large's frozen text encoder (`build_owlv2_vocabulary_bank`), ideal for arbitrary open-world prompts.
- **Adapted Core-27 / Full-37 Banks** (`artifacts/owlv2_30k_{hbb_v3,quad}/owlv2_adapted_vocab_*.pt`): The 27 Core-27 rows receive L2-clamped (`<= 0.12`) visual-text residuals `normalize(t_k + delta_k)` for peak closed-set precision (`86.5%` HBB / `83.6%` Quad Top-1), while unseen or zero-shot prompt vectors remain unmodified on the OWLv2 manifold (`94.7%–97.3%` ZeroShot-4).

---

## 3. Building Custom Vocabularies Offline

Because visual embeddings lie directly in OWLv2-Large's 768-D latent space, **custom vocabularies require zero detector retraining**. Generate a `.pt` bank offline using [`../student_detector/owlv2_teacher.py`](../student_detector/owlv2_teacher.py) and [`../student_detector/vocabulary.py`](../student_detector/vocabulary.py):

```python
import torch
from transformers import Owlv2ForObjectDetection, Owlv2Processor

from student_detector.ontology import CORE_27_CATEGORIES, DISTRACTOR_CATEGORIES
from student_detector.owlv2_teacher import (
    OWLV2_MODEL_ID,
    OWLV2_MODEL_REVISION,
    build_owlv2_vocabulary_bank,
)

custom_categories = [
    "pedestrian", "car", "cyclist", "truck", "bus", "pole",
    "fire hydrant", "cardboard box", "pallet", "forklift", "shopping cart",
    "bollard", "manhole cover", "traffic cone", "construction barrier",
    "backpack", "suitcase",
]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
processor = Owlv2Processor.from_pretrained(OWLV2_MODEL_ID, revision=OWLV2_MODEL_REVISION)
teacher_model = Owlv2ForObjectDetection.from_pretrained(
    OWLV2_MODEL_ID, revision=OWLV2_MODEL_REVISION
).to(device).eval()

# Encodes synonym ensembles via teacher_model.owlv2.get_text_features(...)
custom_bank = build_owlv2_vocabulary_bank(
    teacher_model,
    processor,
    custom_categories,
    device=device,
    synonyms={"forklift": ("forklift", "warehouse lift truck")},
)
custom_bank.save("artifacts/owlv2_custom_17.pt")

# You can also slice any superset bank down to CORE_27_CATEGORIES:
# core27_bank = superset_bank.subset(CORE_27_CATEGORIES)

# Distractor bank (needed once; already tracked at artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt)
distractor_bank = build_owlv2_vocabulary_bank(
    teacher_model, processor, DISTRACTOR_CATEGORIES, device=device
)
distractor_bank.save("artifacts/owlv2_distractors.pt")
```

### Memory & Latency Footprint

| Vocabulary Size `C` | Matrix Shape | FP32 Size on Disk | Edge RAM | Matrix Multiply (4-thr CPU) |
|---|---|---:|---:|---:|
| **27 classes** (`CORE_27_CATEGORIES`) | `[27, 768]` | 83 KB | < 0.2 MB | < 0.05 ms |
| **37 classes** (`OPEN_VOCAB_CATEGORIES`) | `[37, 768]` | 114 KB | < 0.2 MB | < 0.05 ms |
| **128 classes** (custom domain) | `[128, 768]` | 393 KB | < 0.5 MB | < 0.10 ms |
| **500 classes** | `[500, 768]` | 1.54 MB | < 2.0 MB | < 0.35 ms |
| **1,000 classes** (LVIS-scale) | `[1000, 768]` | 3.07 MB | < 4.0 MB | < 0.70 ms |

---

## 4. Edge Runtime Inference & Dynamic Vocabulary Swapping

At inference time on edge hardware, OWLv2-Large and Transformers text encoders are completely absent. Only the detector, semantic head (`head_a` or `head_b`), and compact `.pt` banks are loaded:

```python
import torch

from student_detector.decoder import InferenceDecoder
from student_detector.model import StudentDetector
from student_detector.open_vocab_heads import (
    RoIPooledSemanticHead,
    filter_semantic_detections,
)
from student_detector.targets import point_validity_from_pixel_mask
from student_detector.vocabulary import VocabularyBank

ckpt = torch.load(
    "artifacts/owlv2_30k_hbb_v3/checkpoints/owlv2_hbb_best.pt",
    map_location="cpu",
    weights_only=False,
)
detector = StudentDetector(**ckpt["detector_config"]).eval()
detector.load_state_dict(ckpt["detector_state_dict"])

head_b = RoIPooledSemanticHead.from_config(ckpt["head_b_config"]).eval()
head_b.load_state_dict(ckpt["head_b_state_dict"])

# Attach static 7-distractor bank once for sigmoid background veto
distractors = VocabularyBank.load(
    "artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt", device="cpu"
)
head_b.set_distractor_embeddings(distractors.embeddings)

# Hot-swappable target vocabulary (< 1 ms pointer swap)
vocab = VocabularyBank.load(
    "artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_core27.pt", device="cpu"
)

decoder = InferenceDecoder(
    strides=detector.strides,
    pre_nms_top_k=1000,
    max_detections=100,
    small_box_min_side=16.0,
    small_box_score_power=1.25,
)

# image_t: [1, 3, 384, 384] letterboxed RGB; valid_mask: [1, 384, 384] bool
with torch.no_grad():
    features = detector.forward_features(image_t)
    det_out = detector.forward_eval_head(features, aux_weight=0.0)
    valid_pts = point_validity_from_pixel_mask(valid_mask, detector.strides)
    det = decoder(det_out, (384, 384), valid_point_masks=valid_pts)[0]

    vis_emb = head_b(features, det.boxes, image_shapes=[(384, 384)])
    _logits, probs = head_b.classify_embeddings(vis_emb, vocab.embeddings)
    keep = filter_semantic_detections(det.boxes, det.scores, probs, vocab.categories)

    top_p, cls_ids = probs[keep].max(dim=-1)
    joint_scores = torch.sqrt(det.scores[keep] * top_p)
```

For `QuadProposalDetector`, replace `StudentDetector` + `InferenceDecoder` with [`QuadProposalDetector`](../student_detector/model.py) + [`QuadInferenceDecoder`](../student_detector/quad_decoder.py), pass `[det.boxes]` for RoI pooling, and pass `quads=det.quads` into `filter_semantic_detections`.

---

## 5. Dynamic Context Switching

Because `.pt` vocabulary matrices occupy `< 1 MB`, an application can swap domain vocabularies between frames in `< 1 ms` without reloading the vision backbone:

- **Urban Autonomy (`core27`)**: `owlv2_adapted_vocab_core27.pt` — optimal precision on 27 core navigation/obstacle classes (`street_light` folded into `pole`).
- **Full 37-Class Audit**: `owlv2_adapted_vocab_37classes.pt` — adds scarce infrastructure & furniture classes (`parking_meter`, `stairs`, `wheelchair`, etc.).
- **Custom Open-World Bank**: any bank created by `build_owlv2_vocabulary_bank(...)` (e.g., warehouse, indoor robotics, or inspection prompts).
