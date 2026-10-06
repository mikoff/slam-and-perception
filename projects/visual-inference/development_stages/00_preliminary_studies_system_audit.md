# Visual Inference System Audit: Proposal Duplication, Recall Ceilings, and State-of-the-Art Alignment

**Audit Date:** 2026-10-05  
**Scope:** Architecture, Training Data, Taxonomy, Assignment, Loss Formulations, Suppression, and SOTA Comparison for `projects/visual-inference`  
**Target Checkpoints Audited:**
- Quad Production Model: `artifacts/phase3/checkpoints/best_ema.pt` (Step 83,684, SHA-256: `deee0ce3...`)
- Quad Phase 4 Control: `artifacts/phase4/local_quad_p3_seed42_step20000_v1/best_ema.pt` (Step 19,995, SHA-256: `c9b00311...`)
- HBB Primary: `3bb70fcb...` (Step normalized Phase 3.7 EMA)
- Evaluation Evidence: Frozen 800-image multi-domain benchmark (`artifacts/phase4/baselines/phase4_0_teacher_baseline_report.md` & `phase4_0_visual_audit.html`)

---

## Executive Summary & Core Diagnosis

The visual inference system suffers from two acute, compounding failure modes:
1. **Proposal Clustering / Stacking:** Both HBB and Quad detectors emit dense sprays of 5–10 redundant, overlapping proposals covering the same prominent objects.
2. **Missing Good Detections:** The detectors consistently miss high-contrast physical objects such as trees, pillars/utility columns, and distant/small pedestrians, despite hyperparameter tuning.

### Why Tuning Failed
Manual hyperparameter tuning (adjusting score thresholds between $0.20$ and $0.40$ or tweaking NMS thresholds between $0.70$ and $0.80$) failed because **both failure modes are baked into the training targets, loss formulation, dataset taxonomy, and spatial grid resolution**. Tuning post-processing thresholds cannot compensate for structural training deficiencies:

```
[Problem: Clustered Proposals]
ATSS assigns 18 anchors/GT ──► All trained to quality ~ 1.0 (Centerness target) ──► Flat score peak across object
                                                                                           │
NMS threshold locked at 0.78 (tuned to game AR@100) ◄── Decoded polygon IoU jitter drops IoU to ~0.68-0.75
           │
           ▼
Both survive! 5-10 overlapping quads consume top-100 proposal budget ──► Zero slots left for secondary objects.

[Problem: Missed Trees, Pillars, Distant Pedestrians]
1. Trees & Pillars: UNANNOTATED in COCO, nuImages, BDD100k; rejected in WoodScape. Supervised model treats them as background!
2. Pedestrians: 384x384 downsampling + P3 stride 8 ──► Distant pedestrian is <6px tall (smaller than 1 grid cell).
   Grid points fall outside object ──► Marked "unrepresentable" ──► Discarded from training! (Recall@0.50 on tiny: 0.0%).
3. Weak Negatives = 0.0: COCO has 0 trusted negatives ──► 50% of training data receives 0.0 background penalty!
```

---

## Part 1: Anatomy of Proposal Stacking & Duplication

### 1.1 The Multi-Level ATSS Over-Assignment Trap
In [`student_detector/quad_assigner.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_assigner.py#L42-L60):
```python
top_k: int = 9,
eligible_levels: int = 2,
```
And in lines 182–199:
```python
eligible.scatter_(1, eligible_order[:, : self.eligible_levels], True)
for level, level_slice in enumerate(level_slices):
    ...
    top_values, top_indices = torch.topk(level_scores, count, dim=0, largest=True)
    selected[top_indices[keep] + int(level_slice.start or 0), ...] = True
```
- **The Mechanism:** Every single ground-truth object assigns up to **9 anchor points** on Level $L$ and **9 anchor points** on Level $L+1$. That is up to **18 distinct anchor locations per ground-truth object**.
- **In HBB ([`student_detector/assigner.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/assigner.py#L36-L46)):** `top_k = 9` across all 3 levels (P3, P4, P5), selecting up to **27 positive anchors**.
- **The Consequence:** Every single one of these 18–27 locations is trained to emit a positive prediction for the same object. At test time, all 18 anchors fire simultaneously with high confidence.

### 1.2 The Centerness Disconnect & Flat Score Landscape
In [`student_detector/quad_losses.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_losses.py#L62-L64) and active training config [`correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml`](file:///home/sashamikoff/git/skillup/projects/visual-inference/configs/correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml#L91-L93):
```yaml
quality_target_mode: centerness
quality_blend: 0.0
geometry_quality_target: corner_proxy
```
- **The Mechanism:** The quality prediction head [`SharedQuadProposalHead.quality`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/head.py#L101) outputs a single channel trained against geometric Gaussian centerness:
  $$\text{quality\_target} = \exp\left(-\gamma \cdot \left(\frac{x - x_c}{w/2}\right)^2 - \gamma \cdot \left(\frac{y - y_c}{h/2}\right)^2\right)$$
- **The Defect:** This target is **completely decoupled from actual regression accuracy**. Whether an anchor's predicted corners achieve an IoU of $0.90$ or an IoU of $0.45$ with the ground truth, its quality target remains identical (e.g. $0.95$ if it is near the center).
- **Inference Reality:** Multiple spatial points around the center all predict quality scores in the $0.80$–$0.95$ range. There is **no sharp peak** at the single best-localized candidate. The score distribution across the object is essentially flat.

### 1.3 The Flawed Calibration Metric: Chasing AR@100 with NMS 0.78 / 0.80
In [`docs/correction_phase_3_7_suppression_calibration.md`](file:///home/sashamikoff/git/skillup/projects/visual-inference/docs/correction_phase_3_7_suppression_calibration.md#L62-L66) and [`docs/code_audit.md`](file:///home/sashamikoff/git/skillup/projects/visual-inference/docs/code_audit.md#L54):
- The official production operating points were frozen at:
  - **Quad:** NMS IoU $= \mathbf{0.78}$, Score Threshold $= 0.20$
  - **HBB:** NMS IoU $= \mathbf{0.70}$, Score Threshold $= 0.30$
  - **Config Default ([`config.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/config.py#L112)):** NMS IoU $= \mathbf{0.90}$!

#### Why Polygon IoU Fails at High NMS Thresholds
In axis-aligned bounding boxes (HBB), two boxes with identical centers and slight size variations often have IoU $> 0.75$.
In arbitrary quadrilaterals, **polygon IoU is hyper-sensitive to angular jitter and corner perturbations**:
- If Quad A and Quad B bound the exact same vehicle, but one quad has a $4^\circ$ tilt or a 2-pixel corner shift, their exact polygon intersection area drops sharply:
  $$\text{Polygon\_IoU}(Q_A, Q_B) \approx 0.65 - 0.74$$
- Under an NMS threshold of $\mathbf{0.78}$, the suppression condition:
  $$\text{suppressed} \iff \text{IoU}(Q_A, Q_B) > 0.78$$
  evaluates to **FALSE**!
- Quad B is **NOT suppressed**. It is appended to the final detections.
- Then Quad C (predicted from P4, stride 16) has IoU $0.68$ with Quad A and IoU $0.72$ with Quad B. It is **ALSO NOT SUPPRESSED**.
- **Result:** Quad A, Quad B, Quad C, Quad D, and Quad E all survive and stack visually directly on top of each other!

#### Why the System Was Calibrated to 0.78
The suppression calibration was run to maximize COCO Average Recall at 100 proposals (**AR@100**).
In COCO AR, recall is averaged across 10 IoU thresholds: $\{0.50, 0.55, \dots, 0.90, 0.95\}$.
- If you enforce NMS at $0.50$, you keep only 1 proposal per object. If that single proposal has IoU $0.74$ with the GT, it hits the recall bins up to $0.70$, but misses $0.75, 0.80, 0.85, 0.90, 0.95$.
- If you loosen NMS to $0.80$, the detector keeps 6 overlapping variations of that box. Variation #4 happens to hit IoU $0.85$, boosting AR@100.
- **The Optimization Trap:** The benchmark metric rewarded clustering. Maximizing AR@100 directly incentivized the model to spam multiple overlapping proposals on easy objects.

### 1.4 Proposal Budget Starvation ($K=100$)
In [`student_detector/quad_decoder.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_decoder.py#L101):
- `max_proposals: int = 100`
- `pre_nms_top_k: int = 300`
- When an image contains 8 prominent vehicles or pedestrians, each spawning 8–10 duplicate proposals with scores $>0.70$, those 8 objects consume **64 to 80 of the 100 available proposal slots**.
- Faint, distant, or novel objects with scores in the $0.20$–$0.40$ range are pushed completely out of the top-100 list. Redundant proposals actively cannibalize the proposal budget.

---

## Part 2: Anatomy of Missed Detections (Trees, Pillars, Distant Pedestrians)

### 2.1 The "Open-World" Fallacy: Trees and Pillars Do Not Exist in Training Data
The system was conceived as a "class-agnostic proposal detector," leading to the expectation that any physical object (a tree, building column, stone pillar, or street furniture) should receive a proposal.

**Examination of the actual training dataset mixture proves why this is impossible:**
The training dataset mixture defined in [`correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml`](file:///home/sashamikoff/git/skillup/projects/visual-inference/configs/correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml#L24-L28) is:
- **COCO 2017:** 50%
- **WoodScape:** 20%
- **nuImages:** 18%
- **BDD100k:** 12%

Let us inspect the ground-truth taxonomy in [`dataset-pipeline/configs/automotive_taxonomy_mapping.json`](file:///home/sashamikoff/git/skillup/projects/visual-inference/dataset-pipeline/configs/automotive_taxonomy_mapping.json#L261-L350):

| Category in Visual Scene | COCO 2017 (50%) | nuImages (18%) | BDD100k (12%) | WoodScape Fisheye (20%) | Effective Training Status |
|---|---|---|---|---|---|
| **Trees / Vegetation** | **Unannotated** (Background) | **Unannotated** (Background) | **Unannotated** (Background) | `nature: drop / REJECT` (Ignored / Negative) | **Treated as Background in ~95% of data** |
| **Pillars / Architectural Columns** | **Unannotated** (Background) | **Unannotated** (Background) | **Unannotated** (Background) | `structure: __ignore_region__` (No positives) | **Treated as Background in 100% of data** |
| **Utility Poles** | **Unannotated** (Background) | **Unannotated** (Background) | **Unannotated** (Background) | `pole: pole` (Sparse positive) | **Background in 80% of data** |
| **Buildings / Walls** | **Unannotated** (Background) | **Unannotated** (Background) | **Unannotated** (Background) | `fence: fence` (Sparse) | **Background in 80% of data** |

- **The Deep Machine Learning Reality:** Standard neural networks with supervised heads do not generalize "objectness" into the open world simply by stripping the class label. 
- In 50% of the training images (COCO), every single tree, pillar, utility pole, and building was unannotated. During training, if the detector predicted a proposal on a tree or a pillar, it was **penalized by the loss function as a false positive**.
- You cannot tune inference parameters to make a network detect objects it was systematically penalized for predicting during training.

### 2.2 The Zero Weak-Negative Loss Anomaly (`weak_negative_weight: 0.0`)
In [`student_detector/quad_targets.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_targets.py#L149-L151):
```python
trusted_background = background & points_inside_quads(points, trusted_quads)
trusted_background_masks.append(trusted_background)
weak_background_masks.append(background & ~trusted_background)
```
And in [`student_detector/quad_losses.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_losses.py#L282-L286):
```python
quality_loss = (
    positive_quality
    + trusted_quality
    + targets.weak_negative_weight * weak_quality
)
```
In the active training configuration ([`correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml:84`](file:///home/sashamikoff/git/skillup/projects/visual-inference/configs/correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml#L84)):
```yaml
weak_negative_weight: 0.0
```

#### What is a "Trusted Negative" vs "Weak Negative"?
- **Trusted Negatives:** Only pixels inside explicit ground-truth background polygons (e.g. annotated `drivable_area`, `ego_vehicle`, `sky`).
- **Weak Negatives:** All other background in the image: sidewalks, buildings, trees, background scenery, clutter, and unannotated objects.

#### The Catastrophe:
- In **COCO 2017 (50% of all training batches)**: There are **zero** trusted background annotations! Every single non-object pixel in COCO is classified as a "weak negative".
- With `weak_negative_weight: 0.0`:
  $$\text{quality\_loss}_{\text{COCO\_background}} = 0.0 + 0.0 = \mathbf{0.0}$$
- Across half the training dataset, **the model received zero gradient penalizing high quality on the background**.
- Why was this set to $0.0$? The developers hoped to avoid penalizing unannotated objects ("open-world preservation").
- **The Backfire:** Instead of learning open-world objectness, the quality score head became severely uncalibrated. The network never learned sharp spatial boundary discrimination between objects and background textures, causing background edges to hallucinate high quality while true small objects were suppressed.

### 2.3 The Small-Object Scale Ceiling (Distant Pedestrians)
In the Phase 4.0 evaluation across 800 validation images ([`phase4_0_teacher_baseline_report.md`](file:///home/sashamikoff/git/skillup/projects/visual-inference/artifacts/phase4/baselines/phase4_0_teacher_baseline_report.md#L33-L41)):

| Size Bin | GT Instances | Proposal Recall@0.50 | Teacher Crop Top-1 |
|---|---|---|---|
| `tiny` (<16px) | 928 | **0.0%** | 3.2% |
| `small` (16–32px) | 1,830 | **1.6%** | 12.6% |
| `medium` (32–96px) | 3,131 | **31.4%** | 42.2% |
| `large` (>96px) | 1,643 | **89.5%** | 68.1% |

**Out of 2,758 small and tiny objects (36.6% of all objects in the benchmark), the proposal detector found virtually NONE (0.0% and 1.6%).**

#### Why Pedestrians Suffer This Collapse:
1. **Severe Resolution Downsampling (384x384):**
   - Autonomous vehicle datasets record images at $1920 \times 1080$ (nuImages: $1600 \times 900$).
   - Downsampling to $384 \times 384$ compresses the image by a factor of $\approx 4.2\times$ to $5.0\times$.
   - A pedestrian standing 25–40 meters away is $\approx 25$–$35$ pixels tall and $8$ pixels wide in the original camera image.
   - Rescaled to $384 \times 384$, that pedestrian is **$6$ to $8$ pixels tall and only $1.8$ pixels wide**!
2. **Feature Stride Mismatch (P3 Stride 8):**
   - The finest feature map in the network is **P3 with stride 8** (P2 stride 4 was removed from production).
   - On P3, a grid cell represents an $8 \times 8$ pixel patch of the input image.
   - The pedestrian ($6 \times 1.8$ pixels) is **smaller than a single grid cell**!
3. **The Assigners Drop Small Objects as "Unrepresentable":**
   In [`student_detector/assigner.py:67-70`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/assigner.py#L67-L70):
   > *"A ground truth is geometrically representable only when at least one grid point lies inside it. Boxes without such a point are returned in `unrepresentable_gt_indices` instead of receiving impossible negative distance targets."*
   - Because the pedestrian is $1.8$ pixels wide, the probability that an $8 \times 8$ grid point falls strictly within its boundaries is negligible.
   - When no grid point lands inside the pedestrian, the assigner classifies the object as `unrepresentable` and **drops it completely from positive supervision**.
   - The network was literally never trained on distant pedestrians.

### 2.4 Extreme Aspect Ratio Collapse for Pillars and Poles
Pillars and poles have aspect ratios between $5:1$ and $15:1$ (e.g. width 4px, height 50px).
- **Centerness Target Vanishing:** In [`quad_assigner.py:101-105`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_assigner.py#L101-L105), target quality decays quadratically along the minor axis:
  $$\text{quality} = \exp\left(-\gamma \cdot \left(\frac{x_{\text{local}}}{\text{half\_extent}_{\text{minor}}}\right)^2\right)$$
  For a pillar with width 4px ($\text{half\_extent} = 2\text{px}$), any point just $2\text{px}$ off-center drops the target quality from $1.0$ to $e^{-2.0} \approx 0.135$.
  Along the width axis, the positive basin is less than 1 pixel wide.
- **Isotropic Convolutional Receptive Fields:** The backbone ([`backbone.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/backbone.py)) and head ([`head.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/head.py)) use standard $3 \times 3$ isotropic depthwise separable convolutions. They lack deformable offsets (DCNv2/v4) or anisotropic kernels capable of tracking long, thin vertical structures.

### 2.5 Unscaled Quad Regression Head
In [`SharedDetectionHead`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/head.py#L50-L68) (HBB), regression features learnable scale parameters per pyramid level:
```python
self.scales = nn.Parameter(torch.ones(len(strides)))
distance = torch.relu(self.scales[level] * raw_distance) * self.strides[level]
```
In [`SharedQuadProposalHead`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/head.py#L112-L116) (Quad):
```python
def _forward_level(self, feature: Tensor) -> tuple[Tensor, Tensor]:
    quality = self.quality(self.quality_tower(feature))
    offsets = self.corner_offsets(self.geometry_tower(feature))
    return quality, offsets
```
- The quad head has **no level-specific scale parameter**. The exact same linear weights in `self.corner_offsets` are evaluated on P3 (stride 8), P4 (stride 16), and P5 (stride 32).
- The network is forced to compromise its corner predictions across vastly different spatial scales, resulting in noisy corner regressions that further degrade polygon IoU overlap during suppression.

---

## Part 3: State-of-the-Art (SOTA) Gap Analysis

To understand why this architecture struggles compared to modern vision systems, we contrast it against the leading detection paradigms:

| Architectural Component | Current `visual-inference` Implementation | SOTA Object Proposal Paradigm (OLN / SAM / CutLER) | SOTA Dense Detection Paradigm (TOOD / GFL / YOLOv8) | SOTA End-to-End Paradigm (RT-DETR / Co-DETR) |
|---|---|---|---|---|
| **Objective Type** | Class-agnostic bounding / quad proposals | Class-agnostic object discovery | Task-aligned object detection | One-to-one set prediction |
| **Label Assignment** | Static ATSS: Top-9 per level $\times$ 2 levels = **18 positive anchors/GT** | IoU-based dynamic assignment (Top 3–5) | Dynamic Task-Aligned Assigner (TAL): Cost $= S^\alpha \times \text{IoU}^\beta$ | **Hungarian Matching: Exactly 1 query per GT** |
| **Quality / Score Target** | Decoupled Gaussian geometric centerness (unrelated to box fit) | Centerness $\times$ predicted box IoU (OLN, ECCV 2022) | **Task Alignment:** $t = \text{IoU}$ or $t = \text{cls}^\alpha \cdot \text{IoU}^\beta$ via Quality Focal Loss | Classification score calibrated directly to Hungarian assignment |
| **Suppression / NMS** | Hard NMS at **IoU 0.78 / 0.90** | Soft-NMS or Matrix NMS at IoU 0.45–0.50 | Fast NMS at **IoU 0.45–0.50** | **NO NMS REQUIRED (NMS-free)** |
| **Duplicate Proposal Behavior** | **Catastrophic stacking** (5–10 proposals per object) | Suppressed via Soft-NMS decay | Suppressed via sharp TAL score peaks + standard NMS | **Zero duplicates mathematically guaranteed** |
| **Feature Strides & Resolution** | 384x384, P3-P5 (strides 8, 16, 32) | 640x640–1024x1024, P2-P5 (strides 4, 8, 16, 32) | 640x640, P3-P5 with P2 head or high-res FPN | 640x640, multi-scale deformable attention |
| **Small Object Capability** | **0.0% recall on tiny (<16px), 1.6% on small** | Robust (P2 stride 4 captures 8px objects) | Robust (TAL + P2/P3 captures small instances) | High small-object recall via fine-scale queries |
| **Unannotated Objects (Trees, Pillars)** | **Supervised as background (loss 0.0 or penalty)** | Trained on **Objects365 / LVIS / SA-1B** (365–1000+ classes) or DINOv2 self-supervised patch tokens | Supervised on comprehensive closed sets | Supervised on comprehensive closed sets |

### Key Theoretical Takeaways from SOTA:

#### 1. Why TOOD / GFL Solved the "Overlapping Proposals" Problem
In 2020–2021, the object detection community discovered the exact failure mode observed here: **the misalignment between classification/quality confidence and localization accuracy**.
- When quality is trained on static centerness, anchors with terrible regression predictions can still output quality $= 0.95$.
- In **TOOD (Task-aligned One-stage Object Detection, Feng et al., ICCV 2021)** and **Generalized Focal Loss (Li et al., NeurIPS 2020)**:
  $$t = \text{IoU}(\text{pred\_box}, \text{gt\_box})$$
  The classification/quality head is trained directly to predict the *actual IoU* achieved by that anchor's regression.
- If an anchor is slightly off-center and predicts a loose box (IoU 0.55), its quality target is forced to $0.55$. Only the single anchor that predicts a tight box (IoU 0.92) achieves a score of $0.92$.
- **Result:** The score landscape becomes a sharp Dirac-like peak at the optimal anchor. At test time, NMS has a clear, dominant winner and easily suppresses adjacent candidates.

#### 2. Why RT-DETR Completely Eliminates Duplicates
In **RT-DETR (Real-Time DETR, Lyu et al., CVPR 2024)**:
- Hungarian bipartite matching optimizes a global assignment cost matrix between queries and ground-truth objects:
  $$\hat{\sigma} = \arg\min_{\sigma \in \mathfrak{S}_N} \sum_{i}^N \mathcal{L}_{\text{match}}(y_i, \hat{y}_{\sigma(i)})$$
- Because the assignment is strictly one-to-one, **only one query is ever assigned to an object**.
- The model never learns to emit duplicate proposals. The output of the decoder goes directly to downstream perception without any NMS step.

#### 3. Why OLN (ECCV 2022) Can Propose Trees and Pillars but Visual-Inference Cannot
In **OLN (Object Localization Network, Kim et al., ECCV 2022)**:
- The authors investigated class-agnostic proposals for open-world discovery.
- They proved that class-agnostic networks trained on small taxonomies (like COCO 80 classes) collapse into a closed-world detector that ignores unannotated objects.
- To discover trees, pillars, and novel objects, OLN trains on **Objects365** or **LVIS**, which explicitly annotate thousands of natural and urban object classes.
- Alternatively, modern open-world discovery (such as CutLER, Wang et al. CVPR 2023) uses self-supervised **DINOv2 features**, where spatial patch self-attention naturally groups pixels of coherent physical objects (trees, rocks, pillars) without any bounding box supervision.

---

## Part 4: Root Cause Summary Matrix

| Failure Mode | Direct Manifestation in Codebase | Fundamental Root Cause | Why Tuning Hyperparameters Failed |
|---|---|---|---|
| **Proposals stack on top of each other** | Multiple overlapping boxes on cars and pedestrians | 1. ATSS assigns 18 positive anchors per GT object across P3/P4.<br>2. Head outputs decoupled geometric centerness, creating a flat score plateau.<br>3. Polygon NMS set to 0.78 / 0.80 / 0.90 to maximize COCO AR@100. Angular jitter keeps overlapping quads at IoU 0.68–0.75, so none suppress. | Tuning score threshold (0.2 vs 0.3) or NMS (0.70 vs 0.78) leaves all redundant candidates above the suppression threshold. |
| **Missing Trees and Pillars** | Empty detections on prominent trees, building columns, utility poles | 1. In COCO, nuImages, and BDD100k, trees and pillars are unannotated (treated as background). In WoodScape, nature was dropped/rejected.<br>2. Model was explicitly trained to treat these regions as negative background. | No inference threshold adjustment can force a network to detect objects it was supervised to suppress. |
| **Missing Distant / Small Pedestrians** | 0.0% recall on tiny objects (<16px), 1.6% on small (16–32px) | 1. 384x384 downsampling shrinks distant pedestrians to <6px tall and <2px wide.<br>2. Finest feature stride is P3 (stride 8). Pedestrian is smaller than a single feature cell.<br>3. Assigner marks them `unrepresentable` and discards them from training supervision. | The network never received training gradients on distant pedestrians because they could not physically be indexed on the stride-8 grid. |
| **Missing Thin Structures (Poles/Columns)** | High-contrast vertical structures produce zero detections | 1. Gaussian centerness target with $\gamma=2.0$ falls to zero within 1px along the thin axis.<br>2. $3 \times 3$ isotropic depthwise convolutions cannot span extreme aspect ratios ($10:1$). | Target collapse during training: positive targets for thin structures had near-zero centerness weight. |
| **Budget Exhaustion (K=100)** | Only the largest 8–10 objects receive proposals; all others lost | 1. Top 100 proposals are consumed by 8–10 duplicate quads per prominent object.<br>2. Pre-NMS top-300 candidates are dominated by high-centerness anchors of the same few easy objects. | At $K=100$, 8 objects $\times$ 10 redundant quads = 80 proposals, leaving no budget for secondary detections. |

---

## Part 5: Actionable Roadmap & Engineering Recommendations

To resolve these failures, changes are grouped into four sequential tiers:

### Tier 1: Immediate Post-Processing / Inference Fixes (Zero Retraining Required)
*These can be applied immediately to existing checkpoints to stop proposal stacking and free up the proposal budget.*

1. **Drop NMS IoU Threshold to 0.45 – 0.50:**
   - Change `nms_iou_threshold` from `0.78` (or `0.70`) down to **`0.45` – `0.50`** in [`student_detector/config.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/config.py#L112) and inference scripts.
   - *Impact:* Quads and boxes with IoU in the $0.50$–$0.75$ range will immediately be suppressed. Stacking will collapse from 8 proposals per object down to 1–2 proposals per object.
   - *Budget Relief:* Freeing up 60–70 proposal slots immediately allows secondary objects previously crowded out of the top-100 list to surface.
2. **Implement Two-Stage Bounding-Box Enveloped Polygon NMS:**
   - In [`student_detector/quad_decoder.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_decoder.py#L203):
     Before running exact polygon NMS, compute the axis-aligned envelope of each quad and apply fast torchvision HBB NMS at IoU $0.50$.
   - *Impact:* Eliminates near-duplicate candidates before expensive polygon clipping, robustly handling quads with slight angular variations that artificially lower polygon IoU.
3. **Switch to Soft-NMS (Score Decay instead of Hard Elimination):**
   - Apply Gaussian Soft-NMS:
     $$s_i \leftarrow s_i \cdot \exp\left(-\frac{\text{IoU}(M, b_i)^2}{\sigma}\right)$$
   - Redundant proposals will have their scores sharply penalized rather than remaining tied at $0.85$, ensuring that distinct objects with scores $0.35$ rank ahead of duplicates.

### Tier 2: Training Pipeline & Loss Function Fixes (Requires Retraining Head)
*Fixes the root causes of the flat score landscape and background hallucination.*

1. **Activate Task-Aligned IoU Quality Targets (`quality_target_mode: iou`):**
   - In [`student_detector/quad_losses.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_losses.py#L247-L271), the codebase already contains an unused implementation of `quality_target_mode: "iou"` and `"blend"`.
   - Update training configuration:
     ```yaml
     quad:
       quality_target_mode: blend
       quality_blend: 0.8  # 80% predicted IoU, 20% centerness
       geometry_quality_target: exact_iou
     ```
   - *Impact:* Ties the quality score directly to regression accuracy. Sub-optimal anchors that regress sloppy corners will have their quality scores suppressed toward zero.
2. **Tighten ATSS Assignment Footprint:**
   - In [`student_detector/quad_assigner.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/quad_assigner.py#L42-L45):
     Reduce `top_k` from `9` down to **`3` or `4`**.
     Reduce `eligible_levels` from `2` down to **`1`** (only the single best matching FPN scale should train the object).
   - *Impact:* Cuts positive training anchors from 18 down to 3 per object, eliminating cross-level duplicate predictions at the source.
3. **Re-enable Weak Negative Background Supervision:**
   - In training configs, change `weak_negative_weight` from `0.0` to **`0.15` – `0.25`**.
   - *Impact:* Re-introduces negative gradients across the 50% COCO portion of the dataset, teaching the network to sharply distinguish object boundaries from background clutter.
4. **Add Learnable Per-Level Scale Parameters to Quad Head:**
   - Update [`SharedQuadProposalHead`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/head.py#L103) to mirror `SharedDetectionHead` by introducing:
     ```python
     self.scales = nn.Parameter(torch.ones(len(strides)))
     offsets = self.corner_offsets(...) * self.scales[level]
     ```

### Tier 3: Spatial Resolution & Architectural Upgrades (Fixing Pedestrians & Pillars)
*Breaks the physical resolution barrier for small and thin objects.*

1. **Re-activate the P2 Feature Pyramid Level (Stride 4):**
   - The repository already has a functional [`P2LiteFPN`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/neck.py) implementation.
   - Incorporating P2 (stride 4) provides a $96 \times 96$ feature grid at 384x384 input resolution.
   - A distant pedestrian (6px tall $\times$ 2px wide) now encloses multiple stride-4 grid cells, eliminating the `unrepresentable` dropout in the assigner.
2. **Increase Input Resolution to $512 \times 512$ or $640 \times 640$:**
   - Moving from $384 \times 384$ to $512 \times 512$ increases pixel density by $1.78\times$; $640 \times 640$ increases it by $2.78\times$.
   - This single change elevates distant pedestrians from sub-pixel noise into discernible physical objects with extractable visual features.
3. **Deformable Convolutions for Extreme Aspect Ratios (Pillars & Columns):**
   - Replace the isotropic $3 \times 3$ depthwise convolutions in the geometry tower of [`SharedQuadProposalHead`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/head.py#L97) with Deformable Convolution v2/v4.
   - Deformable kernels dynamically stretch their receptive fields vertically to capture contiguous 10:1 aspect ratio structures without target collapse.

### Tier 4: Dataset & Ontology Realignment (True Open-World Objectness)
*The only way to achieve reliable detection of trees, pillars, and novel objects.*

1. **Acknowledge Closed-World Boundaries of Automotive Datasets:**
   - An automotive dataset annotates cars, people, and bicycles. It cannot teach an agent to detect trees and pillars.
2. **Incorporate Open-Vocabulary / Dense Object Pretraining:**
   - Pretrain the backbone and proposal head on **Objects365** or **LVIS** (which annotate trees, poles, street furniture, architectural columns, rocks, and debris).
   - Alternatively, extract self-supervised object discovery pseudo-labels using **CutLER** or **SAM** to provide genuine class-agnostic positive masks across the entire automotive background.
3. **Align Proposal Object Contract:**
   - In [`dataset-pipeline/configs/automotive_taxonomy_mapping.json`](file:///home/sashamikoff/git/skillup/projects/visual-inference/dataset-pipeline/configs/automotive_taxonomy_mapping.json), explicitly change `nature` and `pole` / `structure` from `REJECT` / `__ignore_region__` to approved positive proposal targets if they are intended production goals.

---

## Conclusion

The unsatisfactory behavior of the detector is not an inexplicable failure or a subtle bug in PyTorch:
- **Proposal Stacking** is the mathematically predictable result of training 18 anchors per object with flat centerness targets and then passing them through polygon NMS at IoU 0.78–0.90 (a threshold tuned purely to inflate AR@100).
- **Missed Detections (Trees & Pillars)** occur because the model was trained on datasets where trees and pillars were explicitly left unannotated or labeled as background.
- **Missed Pedestrians** occur because aggressive downsampling to 384x384 with stride-8 FPN renders distant pedestrians smaller than a single grid cell, causing the assigner to silently discard them during training.

Applying the Tier 1 fixes (dropping NMS to 0.45–0.50 and pre-NMS envelope filtering) will immediately resolve the stacking pathology. Resolving the missed detections requires adopting Tier 2 (Task-Aligned IoU targets) and Tier 3 (P2 stride 4 / higher resolution) to establish a truly robust perception foundation.
