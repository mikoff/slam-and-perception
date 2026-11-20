# Module Purpose & Boundaries

Owns open-world SAM pseudo-label discovery, edge-targeted MobileNetV4 student object detection across Horizontal Bounding Box (HBB) and Oriented Quadrilateral (Quad) geometries, and end-to-end SigLIP-distilled open-vocabulary semantic classification without a runtime text encoder.

# Technical Contracts & Interfaces

- **Input & Canvas**: FP32 RGB `[B, 3, H, W]` letterboxed to `384x384` (`divisible by 32`), with `valid_mask` `[H, W]` and `transform = (scale, off_x, off_y, horizontal_flip)`.
- **Backbone & Bidirectional Neck**:
  - `MobileNetV4Backbone` (`mobilenetv4_conv_medium.e500_r256_in1k`) emits `(C2: 48ch, C3: 80ch, C4: 160ch, C5: 256ch)` at strides `s in (4, 8, 16, 32)`.
  - `P2LitePAN` (`neck_type="pan"`, `fpn_channels=128`) performs top-down + bottom-up depthwise-separable fusion returning `[B, 128, H/s, W/s]`; legacy `P2LiteFPN` (`96ch`) remains loadable.
- **Dual-Head Proposal Detectors (`dual_head=True`, `use_3dmf=True`)**:
  - `StudentDetector` (HBB) predicts `objectness` `[B, 1, H_l, W_l]`, FCOS `ltrb` `[B, 4, H_l, W_l]`, and `centerness` `[B, 1, H_l, W_l]`.
  - `QuadProposalDetector` (Quad) predicts `quality` `[B, 1, H_l, W_l]` and corner offsets `[B, 8, H_l, W_l]`.
  - Primary head embeds `CrossScale3DMaxFilter` (3DMF across adjacent scales + 3x3 spatial window); inference defaults `forward_eval_head(features, aux_weight=0.0)`.
- **Open-Vocabulary Heads (`student_detector/open_vocab_heads.py`)**:
  - **Option A (`DenseAnchorSemanticHead`)**: 2-layer depthwise-separable conv tower on `P2LitePAN` + 1x1 projection to L2-normalized `[N, 768]` via `forward_indexed` or static `forward_dense` (zero RoI ops).
  - **Option B (`RoIPooledSemanticHead`)**: coordinate-free dual-scale `MultiScaleRoIAlign` (`focal_margin=0.15`, `context_margin=0.45`, `output_size=5`) + LayerNorm/GELU MLP to L2-normalized `[N, 768]`.
  - **Calibration (`_ContrastiveClassifierMixin`)**: domain-neutral `DEFAULT_PROMPT_TEMPLATES` (`siglip.py`) + disambiguated `DEFAULT_CATEGORY_SYNONYMS` (`vocabulary.py`, incl. `trash_bin`, `dog`, `stairs`, `debris`) scored against frozen `[C, 768]` `VocabularyBank` embeddings via damped visual-centroid offset (`torch.where(raw <= 0.05, 0.85 * raw.clamp(max=0.016), raw)`), asymmetric median subtraction (`torch.where(med < 0.0, 0.75 * med, 0.36 * med)`), 2.0x-scale distractor margin veto (`+0.004`), and Option A+B blending (`0.30 * p_A + 0.70 * p_B`, ranked by `sqrt(s_obj * p_cls)`).
- **Losses & Target Assignment**:
  - `TaskAlignedAssigner` / `QuadTaskAlignedAssigner`: primary 1-to-1 greedy matching + auxiliary 1-to-many TAL (`top_k=13`), slender-aware gating, and `sqrt(IoU)` quality calibration for `min_side < 16`.
  - `OpenVocabularySigmoidLoss`: primary SigLIP distillation (cosine error + KL over `[37; 7]` bank + symmetric `subspace_mse`/`mean_cos_loss` + `t_margin_loss` + InfoNCE) + compatible human-GT support (`compat`, Seesaw rare mitigation floor `0.75`, federated `SOURCE_ANNOTATED_CATEGORIES` masking where `open_images_subset` is pruned to 5 base classes and unannotated background keeps `0.35` weight).
- **Decoders & Semantic Filtering**: `class_agnostic_nms` and `polygon_nms` (`containment_aware`, `pair_ratios > 0.15` when `class_ids=None`) feed `filter_semantic_detections`, which combines HBB + Quad overlap (`torch.maximum(q_iou, 0.85 * b_iou)`), drops `< 4.5px` micro-specks unless `p >= 0.45`, and runs Pass-0 objectness-aware container suppression preserving `VALID_CONTAINER_PARTS`.

# Active Design Patterns & Decisions

- **Stage-2 Morphism & Semantic Refinement**: `warmstart_detector_from_stage2` expands `96ch` checkpoints into `128ch` `P2LitePAN`; `--semantic-refinement` freezes the detector (`scale=90, bias=-4.40`) to align Option A/B heads without harming open-world localization, with 60/40 WiSE-FT head blending on HBB.
- **Open-World Caching & Balanced Sampling**: `artifacts/cache/open_world_pseudo_gt_8k.pt` (`4,358` objects across `2,833` images, adding `fence`/`stroller` while excluding `stairs` on driving datasets via `prepare_siglip_distill_assets.py`) + `siglip_sam_discovery_8k.pt` (`7,337` images) + SAM ignore boxes (`7,360` images) combine with `+1,698` rare-class images (`8,898` train total, `0` val/holdout overlap, `dog` quota `60`, `bdd100k` specks `< 8px` masked).
- **P2-Aligned Quad GT Retention**: `HumanGTQuadDataset` sets `regular_min_side=2.0`, `thin_major_axis_min=4.0`, `thin_aspect_ratio_min=1.5`, and `thin_area=4.0` to retain slender poles/signs on the `384x384` canvas.
- **Measured Option A / B / A+B Profile (`benchmark_option_a_vs_b.json`)**:
  - **HBB (pre-WiSE-FT blend, `cohort_val_800` / `holdout_210`)**: `Top1`: `OptA=70.1%/70.7%`, `OptB=69.7%/69.4%`, `OptAB=72.6%/72.6%`; `ZeroShot-4` `4-way / 41-way Top5`: `OptB=59.5%/75.6%` (`val`), `61.8%/82.3%` (`holdout`); `PostSemOvlp=2.58/3.09`; `OptA/B Agree=73.3%/73.5%`; `ActiveCats=36/37 (val), 35/37 (holdout)`.
  - **Quad (`cohort_val_800` / `holdout_210`)**: `Top1`: `OptA=63.1%/62.2%`, `OptB=64.5%/61.9%`, `OptAB=65.9%/63.4%`; `ZeroShot-4`: `OptB=70.1%/83.3%` (`val`), `74.4%/76.9%` (`holdout`); `PostSemOvlp=0.69/0.82`; `OptA/B Agree=71.8%/69.9%`; `ActiveCats=36/37 (val), 34/37 (holdout)`.
- **SQLite Determinism**: `quad_train_enriched.sqlite` and `sam_discovery_enriched.sqlite` query with `ORDER BY rowid` and `torch.multiprocessing.set_sharing_strategy("file_system")`.

# Local Constraints & Gotchas

- **Mandatory FP32 Geometry & Loss under AMP**: Canvas `384^2 = 147,456 > 65,504` (FP16 max); promote quad geometry, `_quality_focal_loss`, `OpenVocabularySigmoidLoss` logits, and `train_proposal_decoder` to FP32.
- **Quad Validity Before Border Clamping**: Filter `quad_validity` on unclamped vertices prior to `.clamp(0, W)`.
- **Coordinate-Free Semantic Heads & BatchNorm Integrity**: Never feed normalized box center coords `[xc/W, yc/H]` into semantic heads; restrict WiSE-FT interpolation to conv/linear or semantic heads so BatchNorm stats stay exact.
- **Flip & Padding Invariants**: Pass `horizontal_flip` to `canvas_box_to_native`; slice `category_names` alongside `retained` geometric masks; apply `point_validity_from_pixel_mask(sample.valid_mask, ...)` in decoders so letterbox padding anchors are masked to `-inf`.
- **Synthetic vs. Empirical Centroid Bias**: Keep the `raw <= 0.05` guard inside `compute_prompt_bias` so empirical SigLIP centroids (`<= 0.02`) are damped (`0.85 * raw.clamp(max=0.016)`) while high-dot synthetic test centroids (`0.45`) still subtract in full.
- **Dashboard Visibility Gating**: `_build_sota_preds` caps rendered predictions at `<= 25/image` using joint score `sqrt(s_obj * p_cls)` after `filter_semantic_detections`.
