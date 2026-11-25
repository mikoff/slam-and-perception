# Module Purpose & Boundaries

Owns open-world SAM pseudo-label discovery, edge-targeted MobileNetV4 student object detection across Horizontal Bounding Box (HBB) and Oriented Quadrilateral (Quad) geometries, and end-to-end OWLv2- and SigLIP-distilled open-vocabulary semantic classification without a runtime text encoder.

# Technical Contracts & Interfaces

- **Input & Canvas**: FP32 RGB `[B, 3, H, W]` letterboxed to `384x384` (`divisible by 32`), with `valid_mask` `[H, W]` and `transform = (scale, off_x, off_y, horizontal_flip)`.
- **Backbone & Bidirectional Neck**:
  - `MobileNetV4Backbone` (`mobilenetv4_conv_medium.e500_r256_in1k`) emits `(C2: 48ch, C3: 80ch, C4: 160ch, C5: 256ch)` at strides `s in (4, 8, 16, 32)`.
  - `P2LitePAN` (`neck_type="pan"`, `fpn_channels=128`) performs top-down + bottom-up depthwise-separable fusion returning `[B, 128, H/s, W/s]`; legacy `P2LiteFPN` (`96ch`) remains loadable.
- **Dual-Head Proposal Detectors (`dual_head=True`, `use_3dmf=True`)**:
  - `StudentDetector` (HBB) predicts `objectness` `[B, 1, H_l, W_l]`, FCOS `ltrb` `[B, 4, H_l, W_l]`, and `centerness` `[B, 1, H_l, W_l]`.
  - `QuadProposalDetector` (Quad) predicts `quality` `[B, 1, H_l, W_l]` and corner offsets `[B, 8, H_l, W_l]`.
  - Primary head embeds `CrossScale3DMaxFilter` (3DMF across adjacent scales + 3x3 spatial window); inference defaults `forward_eval_head(features, aux_weight=0.0)`. Both geometries run `8.12–8.13 GFLOPs` detector core (`7.31M` params), `8.60 GFLOPs` E2E Opt-B (`10.18M`, 25 RoIs), and `11.39–11.40 GFLOPs` E2E Opt-A dense (`7.44M`).
- **Open-Vocabulary Heads (`student_detector/open_vocab_heads.py`)**:
  - **Option A (`DenseAnchorSemanticHead`)**: 2-layer depthwise-separable conv tower on `P2LitePAN` + `1x1` projection to L2-normalized `[N, 768]` via `forward_indexed` or static `forward_dense` (zero RoI ops).
  - **Option B (`RoIPooledSemanticHead`)**: coordinate-free dual-scale `MultiScaleRoIAlign` (`focal_margin=0.15`, `context_margin=0.45`, `output_size=5`) + LayerNorm/GELU MLP to L2-normalized `[N, 768]`.
  - **Calibration (`_ContrastiveClassifierMixin`)**: Supports `teacher_space="siglip"` (centroid offset `torch.where(raw <= 0.05, 0.85 * raw.clamp(max=0.016), raw)`, `2.0x` distractor veto `+0.004`) and `teacher_space="owlv2"` (`init_scale=20.0`, `init_bias=-6.5`, `1.5x` distractor veto `+0.02`, `0.25x` top-1 margin), scoring against `[C, 768]` `VocabularyBank` embeddings via `sqrt(s_obj * p_cls)`.
- **Losses & Target Assignment**:
  - `TaskAlignedAssigner` / `QuadTaskAlignedAssigner`: primary 1-to-1 greedy matching (`max_one_to_one_interior_area_frac=0.90` default, `0.20` in OWLv2 HBB/Quad builders so capped `building`/`tree` boxes do not create large negative dead zones) + auxiliary 1-to-many TAL (`top_k=10` HBB / `8` OWLv2 Quad) + slender-aware gating; `ProposalLoss` and `QuadProposalLoss` (`use_proxy_quality_floor=False`) use raw IoU quality targets across all box scales (no `<16px sqrt(IoU)` floor).
  - `OpenVocabularySigmoidLoss`: `"owlv2"` mode pairs native 768-D distillation (`min_side>=10px & gt_iou>=0.40`) over 37+7 axes with Core-27 closed-set focal + hard-negative margin (`street_light->pole`).
- **Decoders & Semantic Filtering**: `InferenceDecoder` and `QuadInferenceDecoder` apply `small_box_min_side=16.0`, `small_box_score_power=1.25`, `pre_nms_top_k=1000`, `max_detections=100` (ranking Quad candidates by clamped envelope while preserving pre-clamp `quad_validity`). `class_agnostic_nms` / `polygon_nms` (`containment_aware`) feed `filter_semantic_detections`, combining HBB+Quad IoU (`maximum(q_iou, 0.85 * b_iou)`), dropping `<4.5px` micro-specks unless `p >= 0.45`, and honoring `VALID_CONTAINER_PARTS` and `BACKDROP_NO_SUPPRESS_CONTAINERS = {"building", "tree", "fence", "canopy"}`.
- **Core-27 Preset (`student_detector/ontology.py`, `VocabularyBank.subset`)**: `CORE_27_CATEGORIES` merges `street_light -> pole` (`CLOSED_MERGE_MAP`) and prunes 9 scarce classes (`PRUNED_SCARCE_CATEGORIES`), while Full-37 and arbitrary unseen prompts remain usable via L2-clamped (`<=0.12`) Core-27 residuals.
- **Annotation Hygiene (`student_detector/annotation_hygiene.py`)**: Non-destructive in-memory filtering across `HumanGTHBBDataset`, `HumanGTQuadDataset`, eval, dashboard overlays, and SAM masking (`sanitize_annotations=True`): deduplicates `nuimages`/`coco_2017` `source_hbb` vs `fitted_quad` doubles (`IoU>=0.50`), collapses fragmented `fitted_quad` pieces (`IoU>=0.30` or `IoMin>=0.70`) back to enclosing `source_hbb`, demotes noisy/nested/`<16px` (`<32px` pole major) `woodscape_rgb_fisheye` fragments to `ignore_region=1`, and preserves raw positive `gt_indices`.
- **Teacher Cache & Reproducibility Pins (`student_detector/owlv2_teacher.py`, `configs/holdout_210_ids.json`)**: `OWLV2_MODEL_ID = "google/owlv2-large-patch14-ensemble"` pinned to `OWLV2_MODEL_REVISION = "95e26936e865f87db1742128404b3c035d47d89d"`; 30k+bollard/pillar WAL->DELETE cache `artifacts/cache/owlv2_teacher_30k_patched.sqlite` (`0/236` overlap with tracked `configs/holdout_210_ids.json`); explicit `torchvision>=0.22.0` in `pyproject.toml`.

# Active Design Patterns & Decisions

Full development pipeline, architectural decision records (`D1`–`D9`), historical progression, and SOTA roadmap live in [`docs/owlv2_pipeline_and_decisions.md`](docs/owlv2_pipeline_and_decisions.md). Active `cohort_val_800 / holdout_210` benchmarks (`p=1.25, tk=1000, max_det=100`, `Core-27 Adapted` unless noted):

- **OWLv2-Large Teacher (`1008x1008`, `437.61M`, `282.55ms` GPU bsz=1, `holdout_210`)**: Core-27 Frozen `R@100=70.0%`, `AP50=20.5%`, `Top-1=86.9% / Top-3=96.8%` (`n=1453`), `mAP@50=31.5%`, `ZS4=96.2%/96.2%` (`Full-37`: `Top-1=84.5%`, `pole=38.1%` vs `69.5%`).
- **Active OWLv2-30k HBB `v3` (`scripts/train_owlv2_hbb.py`, `artifacts/owlv2_30k_hbb_v3/checkpoints/owlv2_hbb_best.pt`, Ep 2, composite `0.5738`)**: `R@100=53.9%/51.6%`, `AP50=13.8%/11.7%`, `OptA=80.9%/82.4%` (`mAP50=16.3%/15.7%`), `OptB=84.0%/86.5%` (`Oracle=80.1%/79.2%`, `mAP50=15.1%/16.6%`), `OptAB=83.8%/86.3%` (`mAP50=15.7%/16.9%`), `ZS4(B)=95.4%/94.7%` (`OptAB=96.1%/97.4%`), `PostOvlp=1.54/2.25`, `E2E OptB=247.3ms CPU / 22.2ms GPU`. Full-37 Opt-B=`82.7%/85.7%` (`mAP50=12.1%/15.8%`). Direct fidelity vs `2,256` teacher boxes: `55.4%` `Recall@50`, `85.2%` Top-1 / `96.8%` Top-3 agreement, `0.8024` mean / `0.8278` median matched 768-D cosine sim.
- **Active OWLv2-30k Quad (`scripts/train_owlv2_quad.py`, warm-started `sota_8k_quad` `strict=True`, `artifacts/owlv2_30k_quad/checkpoints/owlv2_quad_best.pt`, Ep 6, composite `0.5647`)**: `R@100=51.9%/50.6%`, `AP50=17.5%/16.9%`, `OptA=79.7%/78.3%` (`mAP50=15.1%/13.5%`), `OptB=83.8%/83.6%` (`Oracle=77.0%/77.3%`, `mAP50=14.6%/12.1%`), `OptAB=83.6%/82.7%` (`mAP50=15.5%/13.5%`), `ZS4(B)=92.7%/97.3%` (`31w-T5=98.7%/100.0%`), `PostOvlp=0.80/1.27`, `E2E OptB=155.8ms CPU / 22.1ms GPU`. Full-37 Opt-B=`82.4%/82.5%` (`mAP50=11.8%/12.1%`). Joint unseen HBB `v3` + Quad dashboard at `artifacts/owlv2_30k_quad/dashboard_open_vocab_a_b_ab.html`.
- **Legacy SigLIP-8k Baseline**: HBB `R@100=43.2%/43.1%`, `OptB=70.4%/71.3%`, `ZS4=60.2%/61.5%`, `288.7ms` CPU; Quad `OptB=64.5%/61.9%`, `ZS4=70.1%/74.4%`.
- **SQLite Determinism**: Query `quad_train_enriched.sqlite` and `sam_discovery_enriched.sqlite` with `ORDER BY rowid`, flatten WAL journals (`journal_mode=DELETE`) before `immutable=1` reads, and use `torch.multiprocessing.set_sharing_strategy("file_system")`.

# Local Constraints & Gotchas

- **Mandatory FP32 Geometry & Loss under AMP**: Canvas `384^2 = 147,456 > 65,504` (FP16 max); keep quad geometry, `_quality_focal_loss`, `OpenVocabularySigmoidLoss` logits, and `train_proposal_decoder` in FP32.
- **Quad Validity Before Clamping**: Filter `quad_validity` on unclamped vertices prior to `.clamp(0, W)`.
- **BatchNorm & Coordinate-Free Invariants**: Freeze shared BN running stats (`freeze_backbone_batch_norm`) every epoch; never feed `[xc/W, yc/H]` into semantic heads; restrict WiSE-FT interpolation so BN stats stay exact.
- **Flip & Padding Invariants**: Pass `horizontal_flip` to `canvas_box_to_native`; slice `category_names` alongside `retained` geometric masks; mask letterbox padding anchors via `point_validity_from_pixel_mask(sample.valid_mask, ...)`.
- **Centroid Bias Guard & Dashboard Cap**: Preserve the `raw <= 0.05` guard in `compute_prompt_bias` for legacy SigLIP banks; `_build_sota_preds` caps rendered predictions at `<= 30/image` via `sqrt(s_obj * p_cls)` after `filter_semantic_detections`.
