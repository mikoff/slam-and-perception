# Class-Agnostic Quadrilateral Proposal Detector

## Problem Statement

The open-vocabulary detector needs a first-stage network that answers only
"where are the visually distinct objects?" before any SigLIP prompt matching is
performed. The stage must produce a small, fixed-budget set of candidate regions
with high class-agnostic recall, including valid four-corner regions for ordinary
horizontal objects, rotated objects, and perspective-distorted objects.

The existing proposal detector is a useful horizontal-box control, but it cannot
represent general quadrilaterals. Its dataset path also converts source polygons
and masks to horizontal rectangles, discards useful geometry, treats annotation
completeness inconsistently across sources, and evaluates primarily at horizontal
IoU 0.50. Those behaviors cannot establish high open-world quadrilateral proposal
recall.

The proposal stage needs an explicit definition of an object. A positive is a
bounded, independently promptable physical instance. Selected nested automotive
parts are also positive regions. Stuff, textures, shadows, reflections, and
amorphous background are not objects. Ambiguous or incompletely annotated
regions must not silently become negative examples.

## Solution

Build a unified class-agnostic proposal model on the existing MobileNetV4 and
P3-P5 feature pyramid. At each prediction location, the model emits one
localization-quality score and eight signed corner offsets. The decoded result is
a convex four-point candidate region; an HBB is represented by its four
rectangular corners and therefore uses the same output contract as a trapezoid.

Train direct corner geometry for every positive instance. Resolve corner-order
ambiguity by matching predictions against the eight cyclic and reversed
traversals of a canonical clockwise target. Do not sort predicted corners in the
training graph. Rank final proposals by learned localization quality, apply exact
polygon NMS to the highest-scoring candidates, and return at most 100 regions.

Replace destructive HBB conversion with a geometry-aware dataset path. Preserve
source geometry long enough to derive one validated quad per positive instance,
retain explicit stuff as trusted background, retain uncertain regions as ignore,
and write a compact deterministic training manifest. Use an open-world benchmark
configuration for architecture selection and a full-production configuration for
the final proposal checkpoint.

Keep the existing HBB proposal model runnable as the comparison control until the
quad model passes the agreed data, recall, stability, and runtime gates.

### Proposal Execution Overview

```text
+------------------+
| RGB image: H x W |
+--------+---------+
         |
         v
+--------------------------+
| MobileNetV4 shared trunk |
+------------+-------------+
             |
             v
+-----------------------------------+
| P3 / P4 / P5 feature pyramid      |
| strides: 8 / 16 / 32              |
+----------------+------------------+
                 |
                 v
+-----------------------------------+
| Shared quad proposal head         |
|                                   |
| per location:                     |
|   quality:        1 scalar        |
|   corner offsets: 8 signed values |
+----------------+------------------+
                 |
                 v
+-----------------------------------+
| Fixed-shape raw output tensors    |
+----------------+------------------+
                 |
                 v
+===========================================================================+
|                     EXPORTED NEURAL NETWORK ENDS                          |
+===========================================================================+
                 |
                 v
+-----------------------------------+
| Decode dense quad candidates      |
| and rank by learned quality       |
+----------------+------------------+
                 |
                 v
+------------------+    +------------------+    +------------------+
| Keep top 300     | -> | Validate and     | -> | Exact polygon    |
| before NMS       |    | canonicalize     |    | NMS              |
+------------------+    +------------------+    +--------+---------+
                                                       |
                                                       v
                                              +------------------+
                                              | At most 100      |
                                              | candidate quads  |
                                              +--------+---------+
                                                       |
                                                       v
                                              +------------------+
                                              | Future SigLIP    |
                                              | region matching  |
                                              | OUT OF SCOPE     |
                                              +------------------+
```

The neural network ends at fixed-shape dense quality and corner-offset tensors.
Decoding, validation, top-K selection, and polygon NMS remain outside the
exported model graph.

## Implementation Decisions

- The initial object contract includes whole promptable physical instances and
  these nested positive regions: wheel/tire assembly, license plate, mirror,
  door, handle, and screen/display. Nearly identical wheel and tire regions are
  one spatial proposal target with multiple semantic aliases. Contained face,
  head, hand, arm, leg, foot, shoe, and logo annotations are initially ignore
  regions rather than positives or background.

- The detector has one shared proposal head with two outputs: one scalar proposal
  quality and eight corner offsets. It does not have separate HBB and quad
  geometry heads. The existing HBB detector remains a separate runnable control.

- Raw geometry outputs are four pairs of signed offsets relative to the assigned
  FPN grid point and normalized by that level's stride. Final decoded proposals
  are canonical four-point polygons. The final public representation uses eight
  corner coordinates plus one proposal-quality score.

- The project-local direct-corner formulation may be called DQCO, but the name
  does not imply an external standard. Its behavior is completely defined by
  this specification.

- Ground-truth quad vertices are validated and ordered clockwise. The canonical
  target produces eight equivalent traversals: four cyclic starting positions in
  each of the two winding directions. The primary corner loss is the minimum
  stride-normalized Smooth-L1 loss across those eight traversals.

```text
One canonical clockwise target (image x points right, y points down):

           P0----------------P1
             \                 \
              \                 \
               P3----------------P2

Equivalent target traversals presented to the loss:

  clockwise                     reversed
  ------------------------      ------------------------
  P0 -> P1 -> P2 -> P3          P0 -> P3 -> P2 -> P1
  P1 -> P2 -> P3 -> P0          P3 -> P2 -> P1 -> P0
  P2 -> P3 -> P0 -> P1          P2 -> P1 -> P0 -> P3
  P3 -> P0 -> P1 -> P2          P1 -> P0 -> P3 -> P2

  fixed prediction slots Q0..Q3
                |
                v
  Smooth-L1 against all eight target traversals
                |
                v
           take minimum loss

No angular sort is applied to Q0..Q3 inside the training graph.
```

- Predicted vertices are not centroid-sorted during training. Prediction
  canonicalization is a decoder/post-processing operation. This avoids abrupt
  training-time identity changes caused by angular sorting and top-left start
  selection.

- The initial geometry objective contains the cyclic/reversed Smooth-L1 term and
  lightweight scale-normalized validity penalties. Validity covers minimum area,
  minimum edge length, consistent convex cross-product signs, and bow-tie edge
  intersections.

- Distribution Focal Loss and Gaussian Wasserstein Distance are excluded from
  the initial model. DFL is reserved for an ablation if continuous corner
  regression is a demonstrated bottleneck. GWD is reserved for an ablation if
  direct corner loss improves while high-IoU polygon recall stalls. Neither may
  replace the direct corner loss without a new decision.

- The proposal-quality output represents expected localization quality rather
  than binary class foreground probability. Quality supervision starts with
  rotated centerness and transitions to detached exact polygon IoU as predicted
  quads become meaningful. Quality Focal Loss trains the resulting continuous
  target. A scale-normalized cyclic corner-quality proxy is the fallback only if
  the exact-IoU target causes unacceptable measured training overhead or
  compilation instability.

```text
training progress ---------------------------------------------------------->

+----------------------+  +----------------------+  +----------------------+
| Quality warm-up      |  | Quality transition   |  | Mature proposal      |
|                      |  |                      |  | ranking              |
| target =             |  | target =             |  | target =             |
| rotated centerness   |  | blend(center, IoU)   |  | stopgrad(quad IoU)   |
+----------------------+  +----------------------+  +----------------------+

Assignment quality chooses training locations.
Predicted quality ranks decoded proposals.
Detached score targets do not send geometry gradients through polygon IoU.
```

- Exact aligned quad IoU for score targets is computed only for assigned
  positives. A fixed-shape implementation may form the intersection candidates
  from predicted corners inside the target, target corners inside the
  prediction, and the sixteen possible edge intersections. Polygon IoU remains
  mandatory for validation regardless of the training-time score proxy.

- Initial assignment is static and anchor-free. It does not use ATSS virtual
  squares, polygon IoU, ProbIoU, GWD, or prediction-dependent dynamic matching.

- For each ground-truth quad, assignment computes the area centroid, local
  principal axes, projected half-extents, and normalized elliptical distance of
  candidate grid points. Candidates must lie inside the full convex quad. Their
  center quality is an exponential function of squared normalized local
  distance, so it remains positive and naturally handles extreme aspect ratios.

- Candidate assignment combines center quality with smooth FPN scale
  compatibility in log scale. The initial object scale is the square root of
  quad area. Maximum extent is a registered ablation if aspect-ratio metrics show
  that the initial scale sends long objects to levels with insufficient context.
  The primary level and an adjacent boundary level are eligible.

- Assignment keeps a bounded number of central candidates, guarantees a
  deterministic nearest-valid-point fallback, and resolves conflicts by combined
  center and scale quality. Normal assignments, conflicts, fallbacks, and
  unrepresentable instances are logged by level, size, aspect ratio, domain, and
  geometry tier.

```text
Local coordinate frame of one ground-truth quad:

                     minor axis v
                          ^
                          |
               P0---------|-------------P1
                 \       .|.  x  .       \
                  \    x  |  o  x         \
                   \ .  o C o  .           \
                    \   x | o x              \
                     P3---|-------------------P2 --> major axis u
                          |

  C = area centroid
  o = selected central positive
  x = eligible inside-quad candidate not selected
  . = other grid location

For each grid point p and eligible FPN level l:

  point inside full convex quad
               |
               v
  d_norm^2 = (u/a)^2 + (v/b)^2
               |
               v
  q_center = exp(-gamma * d_norm^2) --------------------+
                                                            |
  object scale + level reference                            |
               |                                            |
               v                                            v
  smooth log-scale compatibility S(g, l) ---------------> multiply
                                                            |
                                                            v
                                       A(g, p, l) = q_center * S
                                                            |
                                                            v
                                       bounded candidates + fallback

There is no shrunken center polygon, virtual square prior, or assignment IoU.
```

- The initial feature pyramid remains P3, P4, and P5 at strides 8, 16, and 32.
  A stride-4 P2 level is not part of the baseline. It is the first architectural
  ablation if 16-32 pixel recall or assignment audits identify insufficient
  spatial resolution.

- The hard deployment budget is 100 proposals per image. Validation also
  measures 50 and 300 proposals, with 300 serving as a diagnostic ceiling rather
  than a deployment output.

- The primary supported size tier contains objects whose shortest side is at
  least 16 pixels after detector input resizing. Objects from 8 through 16 pixels
  form a difficult diagnostic tier. A genuinely thin, elongated object may
  have a shorter minor axis: it remains positive when its major axis is at
  least 8 pixels, aspect ratio is at least 3:1, and area is at least 16 square
  pixels. Objects failing both the regular and thin-object gates are excluded
  from positive training and headline recall, become ignore rather than
  background, and still have their prevalence reported.

- The initial data scope is limited to the four installed sources: COCO 2017,
  nuImages, BDD100K, and WoodScape RGB Fisheye. Additional general, automotive,
  fisheye, aerial, or oriented datasets are added only in response to a measured
  recall gap.

- Dataset conversion no longer rewrites every retained annotation as a
  rectangle. It preserves polygon and bitmap geometry through taxonomy filtering
  and derives one quad per positive instance.

- Duplicate geometry representations of the same source instance are merged.
  Polygon or mask geometry is preferred as the fitting source, while a source
  rectangle is retained as deterministic fallback metadata. Spatially
  near-identical cross-label part annotations may merge into one proposal target
  with semantic aliases; genuine parent/part regions remain separate.

- A source quad is preserved only when it is finite, convex,
  non-self-intersecting, nondegenerate, and semantically represents the instance.
  A bow-tie caused only by vertex ordering may be repaired when its convex hull
  still contains four vertices. Concave or degenerate four-point inputs are not
  silently converted by inventing vertices.

- Masks and longer polygons use a coverage-first convex quad fit. A fitted quad
  must contain at least 98 percent of the visible instance. Among acceptable
  candidates, lower quad area is preferred to reduce background. Failed fits
  fall back first to a validated minimum-area rotated rectangle and then to the
  HBB.

```text
+------------------------+    +-------------------------+
| Immutable raw geometry | -> | Merge duplicate source |
| and source metadata    |    | representations         |
+------------------------+    +------------+------------+
                                            |
                                            v
                                  +---------------------+
                                  | Select source type  |
                                  +----------+----------+
                                             |
       +-------------------------------------+--------------------------------+
       |                                     |                                |
       v                                     v                                v
  source HBB                         source 4-point set              mask / long polygon
       |                                     |                                |
       v                                     v                                v
  rectangular quad                  valid convex quad?                fitted quad covers
  tier = HBB                         |              |                  >= 98% of instance?
       |                         yes |          no  |                  |              |
       |                             v              |              yes |          no  |
       |                      tier = source_quad    |                  v              |
       |                             |              |           tier = fitted_quad   |
       |                             |              |                  |              |
       |                             |              +--------+         |      +-------+
       |                             |                       |         |      |
       |                             |                       v         |      v
       |                             |                 +-------------------------+
       |                             |                 | fallback target         |
       |                             |                 | valid min-area rect     |
       |                             |                 | otherwise source HBB    |
       |                             |                 +------------+------------+
       |                             |                              |
       +-----------------------------+------------------------------+
                                             |
                                             v
                                  +-------------------------+
                                  | Compact training record |
                                  | bbox + quad + tier      |
                                  | coverage + tightness    |
                                  | state + source metadata |
                                  +-------------------------+
```

- Every derived target records its geometry tier, fit coverage, fit tightness,
  validity, source category, source annotation identity, and semantic aliases.
  Geometry tiers distinguish preserved source quads, fitted quads, rotated
  rectangles, and HBB fallbacks.

- The final training manifest is compact. It stores the derived HBB, quad,
  geometry metrics, supervision state, and source metadata. Full source
  contours and masks remain immutable in raw/intermediate data. Only compact
  background and ignore geometry required by the loader is carried into the
  training view.

- Spatial supervision has four states: positive, trusted background,
  weak/unlabeled, and ignore. Positives receive quality and corner losses.
  Trusted background receives full negative quality supervision.
  Weak/unlabeled space may receive an empirically selected relative negative
  contribution. Ignore regions receive no loss.

```text
+--------------------+---------------------------+--------------------------+
| supervision state  | examples                  | training contribution    |
+--------------------+---------------------------+--------------------------+
| positive           | whole object; allowlisted | quality + corner losses  |
|                    | nested part               |                          |
+--------------------+---------------------------+--------------------------+
| trusted background | explicit sky, road,       | full negative quality    |
|                    | vegetation, other stuff   | loss                     |
+--------------------+---------------------------+--------------------------+
| weak / unlabeled   | image space with unknown  | group-normalized,        |
|                    | annotation completeness   | empirically weighted     |
+--------------------+---------------------------+--------------------------+
| ignore             | withheld class, crowd,    | no loss                  |
|                    | ambiguous or invalid quad |                          |
+--------------------+---------------------------+--------------------------+

Resolution when states overlap at one location:

  assigned positive?  yes --> positive loss; never treat it as background
          |
          no
          v
  inside ignore?       yes --> no loss
          |
          no
          v
  trusted background?  yes --> full negative quality loss
          |
          no
          v
  weak / unlabeled         --> empirical weak-negative contribution
```

- Stuff annotations excluded by the object contract, such as sky, road surface,
  and vegetation, are preserved as trusted-background masks after positive and
  ignore geometry is subtracted. Withheld benchmark categories, crowds,
  ambiguous groups, invalid geometry, and uncertain truncations become ignore
  regions. Assigned positives always override background masks.

- The conversion pipeline labels supervision states and reports their geometry;
  it does not bake numerical loss weights into annotations.

- Quality losses are normalized separately for positives, trusted background,
  and weak/unlabeled locations. The weak-negative contribution is selected
  empirically, including a zero-weight candidate, rather than preset to 0.05 per
  location. Weak-negative sampling or contribution capping prevents the group
  from dominating merely because it has more grid locations.

- Training logs the current group weights and schedules, location counts, raw
  group-normalized losses, weighted loss contributions, weighted-to-positive
  ratios, score quantiles, and per-domain epoch summaries. Periodic shared-head
  gradient contributions are logged because equal loss magnitudes do not imply
  equal optimization effects. Checkpoints store the complete configuration and
  active schedules.

- Mild full-image projective augmentation is applied online to HBB-heavy
  training sources at configurable probability and strength. The same homography
  transforms the image, positive quads, background geometry, and ignore geometry.
  Validation is never projectively warped. Synthetic and real-quad results are
  reported separately.

- Homography augmentation is not presented as fisheye simulation. Fisheye
  behavior is learned from real fisheye data or a separately justified
  distortion-aware transform.

- Geometric transforms use polygon clipping at image boundaries. Coordinate-wise
  clamping of quad corners is forbidden because it can change topology and
  create invalid targets. Exact positive/ignore visibility thresholds remain
  configurable and are selected after transformed-data audits.

- Dataset exports contain the full, deterministic union of accepted data. Source
  and domain sampling weights live in training configuration rather than being
  embedded through destructive resampling.

- Source weights are chosen only after an inventory of eligible positives,
  geometry tiers, size tiers, part status, and trusted-background coverage.
  Training uses deterministic domain-aware batches, logs requested and realized
  image/instance/assignment proportions, and limits excessive repetition of
  smaller sources. Mixtures are selected using unseen and worst-domain recall,
  not aggregate loss alone.

- There are two training views. Open-world benchmark mode uses category-disjoint
  supervision, beginning with the established COCO VOC-20 seen versus COCO-60
  unseen protocol. Withheld instances in training images are ignore regions, and
  validation reports seen and unseen recall separately. Full-production mode
  trains on every accepted positive annotation after architecture selection.

```text
+-------------------------------+    +-------------------------------+
| OPEN-WORLD BENCHMARK VIEW     |    | FULL-PRODUCTION VIEW          |
+-------------------------------+    +-------------------------------+
| seen categories -> positive   |    | every accepted object         |
| unseen boxes    -> ignore     |    | annotation -> positive        |
|                               |    |                               |
| selects architecture, loss,   |    | trains final Stage-1          |
| assignment, and data policy   |    | proposal checkpoint           |
+---------------+---------------+    +---------------+---------------+
                |                                    |
                v                                    v
     seen AR + unseen AR                 all-object and domain AR
```

- An automotive-specific unseen-object and nested-part benchmark may follow the
  standard COCO benchmark once its label coverage is audited. It does not replace
  the standard cross-category control.

- The reference decoder ranks dense P3-P5 predictions, decodes and validates the
  top 300 quads, canonicalizes winding, applies class-agnostic exact polygon NMS,
  and returns no more than 100 proposals. HBB NMS is not the reference behavior.

- NMS and model inference are timed separately. Validation reports recall before
  and after NMS, removed proposal counts, and ground-truth instances lost only by
  suppression. Approximate or compiled NMS is considered only after reference
  recall and runtime are measured.

- Primary selection uses class-agnostic Average Recall at 100 proposals averaged
  over polygon IoU thresholds from 0.50 through 0.95. Diagnostics include recall
  at 50, 100, and 300 proposals at polygon IoU 0.50 and 0.75, plus matched-IoU
  distributions.

- Metrics are sliced by seen/unseen status, source, domain, size, aspect ratio,
  whole object versus nested part, geometry tier, real versus synthetic quad,
  and fisheye radial position. Final numerical pass thresholds are set only after
  measuring the HBB control, quad target quality ceiling, and annotation ceiling.

## Testing Decisions

- Tests assert externally observable behavior at the highest practical seam.
  They do not assert private helper call order, internal tensor temporary shapes,
  or a particular fitting algorithm when multiple algorithms satisfy the same
  manifest contract.

- The primary data seam is raw synthetic Supervisely-style annotations through a
  validated compact proposal manifest. One integration fixture covers rectangle,
  valid quad, long polygon, bitmap, duplicate representations, trusted stuff,
  ignore regions, nested parts, invalid geometry, and deterministic fallbacks.

- Data integration tests verify deterministic and idempotent output, immutable
  raw input, correct supervision states, clockwise valid quads, at least 98
  percent fit coverage, correct geometry-tier reporting, semantic alias merging,
  and valid image links.

- Geometry-conversion tests cover rotated rectangles, trapezoids, extreme aspect
  ratios, bow-tie ordering repair, concave and degenerate inputs, boundary
  clipping, and fallback escalation. Coverage and tightness are verified against
  source masks or polygons rather than against implementation intermediates.

- Transform tests verify that images, quads, background masks, and ignore masks
  receive the same affine or projective transform. They verify convexity,
  visibility-state behavior, deterministic seeded augmentation, and the absence
  of coordinate-wise corner clamping.

- The primary model seam is a small manifest-backed train/validate run that
  produces decoded proposals and class-agnostic metrics. A micro-overfit test
  must demonstrate that the model can learn several rectangular, rotated, and
  trapezoidal instances before any full run is authorized.

- Model contract tests verify fixed P3-P5 raw outputs, one quality channel, eight
  signed offset channels, finite decoding, canonical output quads, and continued
  compatibility of the retained HBB control.

- Assignment tests verify invariance to winding and cyclic start, behavior on
  10:1 aspect-ratio objects, smooth adjacent-level compatibility, point-inside-
  quad eligibility, deterministic conflict resolution, and the guaranteed
  fallback for otherwise unrepresentable eligible instances.

- Loss tests verify identical loss for all eight equivalent target traversals,
  finite gradients for valid and empty batches, scale/stride normalization,
  increased validity penalty for collapsed or self-intersecting predictions,
  and no dependence on sorting predicted corners.

- Quality curriculum tests verify pure centerness at its start, pure detached
  polygon IoU at its end, deterministic interpolation, no geometry gradient
  through the detached score target, and correct fallback-proxy behavior.

- Supervision-loss tests verify independent group normalization, positive
  override of background, zero ignore contribution, configurable weak-negative
  contribution, and complete logging of raw and weighted group statistics.

- Polygon-IoU tests compare the batched aligned implementation against a trusted
  reference on disjoint, contained, identical, edge-touching, rotated,
  perspective, thin, and invalid inputs.

- Decoder tests verify exact polygon suppression behavior for rotated objects
  whose HBBs overlap strongly, preservation of distinct nested regions, invalid
  quad rejection, deterministic score ordering, top-300 preprocessing, and the
  final 100-proposal cap.

- Evaluation tests verify Average Recall across the full IoU range, fixed-budget
  recall, pre/post-NMS recall, all required metric slices, empty-image behavior,
  and strict separation of seen and unseen benchmark annotations.

- Dataset inventory and training logging tests verify requested versus realized
  sampling proportions, every required supervision/geometry statistic, active
  schedule values, and checkpoint reproducibility metadata.

- Performance checks benchmark data conversion throughput, loader throughput,
  aligned polygon-IoU overhead, model forward time, and polygon-NMS time as
  separate measurements. Performance optimizations must preserve the reference
  behavioral tests.

- Existing synthetic dataset-pipeline tests, detector geometry tests, assignment
  tests, decoder tests, evaluation tests, and training smoke tests are prior art.
  They should be extended at their current public seams rather than replaced by
  many tests of new private helpers.

## Out of Scope

- SigLIP vision or text encoders, region embeddings, prompt matching, semantic
  distillation, homography-based teacher crops, and all Stage-2 semantic losses.

- Adding new public datasets during the first implementation.

- Making P2 part of the initial exported model.

- DFL, GWD, ProbIoU-based regression, dynamic assignment, task-aligned
  assignment, and separate HBB-plus-quad refinement heads in the baseline.

- Treating human body parts or logos as initial positive proposal targets.

- Final INT8 quantization, ExecuTorch/TensorRT/NPU lowering, or target-device
  kernel optimization.

- An optimized production polygon-NMS kernel. This spec requires a correct
  reference implementation and profiling boundary.

- Choosing final loss weights, weak-negative contribution, dataset mixture,
  projective augmentation magnitude, assignment scale boundaries, quality
  curriculum duration, or absolute recall pass thresholds before the required
  audits and control runs exist.

- Deleting or silently replacing the existing HBB comparison model.

## Further Notes

- High recall is always interpreted under the fixed proposal budget. Recall
  improvements obtained only by exceeding 100 deployed regions do not satisfy
  the product objective.

- The 98 percent fit-coverage floor concerns visible source-instance geometry.
  Tightness is a separate measure of included background. A rectangle fallback
  makes high coverage achievable; conversion must not trade away visible object
  pixels merely to obtain a visually tighter quad.

- Open-world proposal quality cannot be established by collapsing all labels to
  foreground and validating on the same category distribution. Category-disjoint
  and cross-domain measurements are part of the feature, not optional research
  reporting.

- The HBB control and quad candidate must consume equivalent supervision states
  and source mixtures wherever their geometry contracts permit. Otherwise their
  comparison is not interpretable.

### Gate Semantics

- A **hard gate** blocks all dependent work until its required evidence passes.
  A failed gate is fixed or explicitly redesigned; it is not waived by a good
  aggregate training loss.

- A **calibration gate** records the HBB control, target-quality ceiling, and
  annotation ceiling, then freezes numerical benchmark thresholds before the
  corresponding quad candidate is trained. Thresholds may not be moved after
  candidate results are inspected without invalidating and rerunning the
  comparison.

- A **branch gate** chooses between two already approved implementations from
  measured evidence. The score-target branch may select detached exact polygon
  IoU or the cyclic corner-quality proxy. It does not authorize GWD or a dynamic
  assigner.

- Every gate produces a machine-readable report containing the data-manifest
  fingerprint, configuration, checkpoint identity when applicable, software
  revision, seed, metric definitions, and pass/fail result.

### Consolidated Implementation Plan

The quad detector is introduced beside the existing HBB path. Shared backbone,
FPN, batching, checkpointing, and training infrastructure are reused where their
contracts do not depend on box geometry. Geometry-specific head, assignment,
loss, decoding, and evaluation behavior remains explicit; the HBB control is not
silently changed to make the comparison pass.

```text
                              +----------------------+
                              | P0. Freeze contracts |
                              | and HBB control      |
                              +----------+-----------+
                                         |
                 +-----------------------+-----------------------+
                 |                                               |
                 v                                               v
      +----------------------+                        +----------------------+
      | P1. Quad-aware data  |                        | P2. Reference quad   |
      | conversion           |                        | geometry             |
      +----------+-----------+                        +----------+-----------+
                 |                                               |
                 +-----------------------+-----------------------+
                                         |
                                         v
                              +----------------------+
                              | P3. Loader, states,  |
                              | and transforms       |
                              +----------+-----------+
                                         |
                                         v
                              +----------------------+
                              | P4. Static point     |
                              | assignment + targets |
                              +----------+-----------+
                                         |
                    +--------------------+--------------------+
                    |                                         |
                    v                                         v
         +----------------------+                   +----------------------+
         | P5. Quad head, loss, |                   | P6. Decoder, polygon |
         | quality curriculum   |                   | NMS, and evaluation  |
         +----------+-----------+                   +----------+-----------+
                    |                                         |
                    +--------------------+--------------------+
                                         |
                                         v
                              +----------------------+
                              | P7. Training and     |
                              | observability        |
                              +----------+-----------+
                                         |
                                         v
                              +----------------------+
                              | P8. Micro-overfit    |
                              +----------+-----------+
                                         |
                                         v
                  +----------------------+----------------------+
                  | P9. HBB control vs quad open-world trial    |
                  +----------------------+----------------------+
                                         |
                                         v
                              +----------------------+
                              | P10. Full-production |
                              | Stage-1 checkpoint   |
                              +----------+-----------+
                                         |
                                         v
                              +----------------------+
                              | Stage-2 handoff      |
                              | contract only        |
                              +----------------------+
```

1. **P0 — Freeze contracts and establish the control.** Freeze the object
   policy, quad record schema, supervision states, raw output contract, proposal
   budgets, metric definitions, benchmark splits, and deterministic run
   metadata. Run the unchanged HBB path on the frozen validation views. Audit
   raw annotations and derived target ceilings, then freeze the numerical
   selection thresholds used at P9.

2. **P1 — Build quad-aware conversion.** Replace destructive rectangle-only
   conversion with source-geometry preservation, duplicate merging, source-quad
   validation, coverage-first fitting, rotated-rectangle and HBB fallbacks, and
   compact manifest export. Produce per-source inventory, coverage, tightness,
   geometry-tier, size-tier, part, background, ignore, and failure reports.
   Preserve the current HBB export as a reproducible compatibility view.

3. **P2 — Build the reference geometry layer.** Implement canonical winding,
   all eight equivalent target traversals, area and centroid operations,
   convexity and self-intersection checks, point-in-convex-quad tests, clipping,
   aligned exact quad IoU, and exact reference polygon NMS. Begin with clear CPU
   reference behavior; batch or compile only after parity tests exist.

4. **P3 — Extend loading, supervision, and augmentation.** Load compact quads and
   four-state supervision without reconstructing source contours. Apply the same
   resize, crop, flip, affine, or projective transform to images, positive
   quads, trusted background, and ignore geometry. Clip polygons at boundaries,
   resolve state overlap in the specified order, and collate variable instance
   counts without changing raw model tensor shapes.

5. **P4 — Implement static scale-aware point assignment.** Compute local
   principal axes, normalized elliptical center distance, full-quad inclusion,
   smooth log-scale level compatibility, bounded candidates, deterministic
   conflicts, and nearest-valid-point fallback. Emit assignment diagnostics and
   eight-coordinate stride-normalized corner targets. Do not call polygon IoU in
   assignment.

6. **P5 — Add the quad head and objectives.** Add the one-channel quality output
   and unmasked eight-coordinate signed-offset output to the shared proposal
   head. Implement cyclic/reversed Smooth-L1 geometry loss, lightweight validity
   penalties, group-normalized quality loss, and the centerness-to-detached-IoU
   quality curriculum. Keep the HBB head and losses independently runnable.

7. **P6 — Add reference decoding and evaluation.** Decode dense signed offsets,
   reject invalid candidates, canonicalize accepted quads, rank by learned
   quality, retain 300 before NMS, apply exact class-agnostic polygon NMS, and
   emit at most 100 proposals. Add polygon Average Recall, fixed-budget recall,
   pre/post-NMS recall, target-ceiling metrics, and every required slice.

8. **P7 — Integrate training and observability.** Wire quad records through the
   existing optimizer, EMA, scheduling, checkpoint, sampling, and validation
   lifecycle. Log supervision-group contributions, active curriculum and weak
   negative weights, assignment coverage, fallbacks, geometry validity, score
   calibration, gradients, requested versus realized mixtures, throughput, and
   separate inference/NMS timing.

9. **P8 — Prove learnability with micro-overfit.** Disable stochastic
   augmentation and train a small, fixed fixture containing HBB-derived
   rectangles, rotated rectangles, trapezoids, long thin objects, nested parts,
   ignore regions, trusted background, and an empty image. Do not authorize a
   full run until geometry and ranking both overfit this fixture.

10. **P9 — Run the controlled open-world comparison.** Train the HBB control and
    quad candidate with matched data views, source mixtures, backbone state,
    proposal budgets, schedule, and evaluation images. Compare the frozen
    primary and sliced metrics. Select architectural or policy ablations only
    from a diagnosed failed slice; do not bundle several changes into one trial.

11. **P10 — Train and package the full-production Stage-1 model.** Retrain the
    selected quad configuration with every accepted positive category, publish
    its reproducibility bundle and benchmark report, and validate the fixed
    candidate-region handoff contract. SigLIP integration starts only after this
    gate and belongs to a separate specification.

### Acceptance Gates

#### G0 — Contract, Control, and Threshold Freeze (calibration gate)

Pass only when:

- the current HBB tests and verification run pass unchanged;
- one reproducible HBB report exists for the frozen full-production and
  category-disjoint validation views at 50, 100, and 300 proposals;
- target-quality and annotation-ceiling reports exist for overall, unseen,
  source, domain, size, aspect-ratio, part, and geometry-tier slices;
- primary `AR@100[0.50:0.95]`, diagnostic recall, allowable slice regression,
  and required quad-gain thresholds are numerically frozen from those reports;
- train/validation identity overlap and open-world category leakage are zero;
- the object policy, schema version, proposal budgets, and evaluation protocol
  are fingerprinted and immutable for the comparison.

#### G1 — Dataset and Manifest (hard gate)

Pass only when:

- repeated conversion of the same inputs is byte-deterministic and idempotent,
  raw annotations are unchanged, and every final image link is valid;
- 100 percent of emitted records satisfy the schema and every accepted positive
  has a finite, convex, non-self-intersecting clockwise quad with positive area;
- at least 98 percent of eligible positive source instances reach an accepted
  geometry tier; every remainder is reported and becomes ignore, never negative;
- every fitted quad contains at least 98 percent of its visible source mask or
  polygon, while tightness is reported independently;
- duplicates, aliases, nested positives, stuff background, crowds, withheld
  categories, tiny objects, and invalid geometry resolve to the specified state;
- a stratified visual audit covers at least 300 instances across all four
  sources and every populated geometry tier, with no unresolved systematic
  conversion error.

The two 98-percent checks are distinct: one is aggregate eligible-instance
representability; the other is per-fitted-quad visible-geometry containment.

#### G2 — Geometry Reference (hard gate)

Pass only when:

- all eight cyclic/reversed forms give equal corner loss within declared
  floating-point tolerance and no predicted-corner sort occurs in training;
- quad validation, point inclusion, clipping, fitting checks, and polygon NMS
  pass the synthetic rectangle, rotation, perspective, thin-object, boundary,
  nesting, bow-tie, concave, degenerate, empty, and duplicate cases;
- aligned polygon IoU agrees with the trusted CPU reference to maximum absolute
  error `1e-5` in float64 and `1e-4` in float32 for valid test fixtures;
- valid, invalid, and empty batches produce finite outputs and finite gradients
  wherever gradients are defined;
- optimized geometry paths, if introduced, pass the same public reference tests.

#### G3 — Loader, Supervision, Assignment, and Targets (hard gate)

Pass only when:

- seeded transforms are deterministic and apply one geometry transform to the
  image and every supervision state, with polygon clipping rather than
  coordinate-wise corner clamping;
- positives override background, ignore contributes no loss, trusted background
  contributes full group-normalized negative loss, and weak/unlabeled space uses
  only the configured empirical contribution;
- assignment is invariant to target winding and cyclic start, uses no virtual
  square or polygon-IoU ranking, and passes the 10:1 aspect-ratio fixtures;
- every eligible representable instance receives at least one positive after
  deterministic fallback, conflicts produce one deterministic owner, and every
  unrepresentable instance is reported rather than trained as background;
- assignment counts, levels, conflicts, fallbacks, and unrepresentable cases can
  be reconciled exactly with the emitted training targets.

#### G4 — Model, Loss, and Export Contract (hard gate)

Pass only when:

- P3, P4, and P5 each emit fixed-shape raw tensors containing exactly one
  quality channel and eight unmasked signed corner-offset channels per location;
- decoding those tensors yields finite coordinates and the model graph itself
  contains no top-K, canonicalization, clipping, IoU, or NMS operation;
- geometry, validity, and quality losses are finite for positive, background-
  only, ignore-only, weak-only, and empty batches;
- the quality schedule starts at pure rotated centerness, ends at pure detached
  geometry quality, and sends no score-target gradient into corner geometry;
- complete configuration and schedule state round-trip through checkpoints;
- all retained HBB control tests continue to pass through its separate path.

#### G5 — Score-Target Cost (branch gate)

Benchmark detached aligned exact polygon IoU against the approved cyclic
corner-quality proxy on identical batches after the geometry path is stable.
Select exact IoU only when it compiles and runs without instability, adds no more
than 10 percent to median training-step time, and adds no more than 10 percent to
peak training memory relative to the proxy. Otherwise select the proxy, record
the evidence, and retain exact polygon IoU for validation. This branch is not a
failure and does not permit assignment-time polygon IoU or GWD.

#### G6 — Mixed-Geometry Micro-Overfit (hard gate)

Pass only when the fixed fixture:

- reaches `R@100 IoU=0.50 = 1.00` and `R@100 IoU=0.75 >= 0.95` using decoded,
  ranked, post-NMS proposals rather than oracle matching;
- matches at least one instance from every populated positive geometry tier and
  object-contract condition represented in the fixture;
- shows decreasing corner error and increasing proposal quality without NaN,
  infinite loss, collapsed quads, or uncontrolled validity penalties;
- reproduces its final metrics from the saved checkpoint and configuration.

#### G7 — HBB-versus-Quad Open-World Selection (hard gate)

Pass only when:

- HBB and quad reports use identical validation images, category policy, data
  mixture, proposal budgets, and metric implementation;
- the quad model satisfies every numerical overall, unseen, required-gain, and
  allowable-regression threshold frozen at G0 for primary
  `AR@100[0.50:0.95]` and the required diagnostic slices;
- recall is reported before and after polygon NMS, and suppression-only losses
  do not exceed the frozen allowance;
- the 100-proposal result passes independently of any improvement visible only
  at 300 proposals;
- runtime reports separate data loading, assignment, loss, model forward,
  decoding, and polygon NMS, with no unexplained bottleneck or instability;
- the selected result is reproducible from its recorded manifest, configuration,
  checkpoint, and seed.

If G7 fails, the HBB model remains the active control. The quad path is diagnosed
by failed slice and returns to the smallest relevant earlier phase; it does not
advance to full-production training.

#### G8 — Full-Production Stage-1 Release (hard gate)

Pass only when:

- training uses every accepted production positive and the selected, frozen
  geometry, supervision, assignment, loss, and sampling policy;
- the released checkpoint reproduces its report and emits at most 100 valid,
  scored, canonical candidate quads per image through the reference decoder;
- all headline and sliced metrics, target ceilings, throughput, forward time,
  polygon-NMS time, and peak memory are published with the release bundle;
- checkpoint metadata contains the complete manifest/configuration fingerprints,
  software revision, schedules, seed, and model output schema version;
- the HBB control remains runnable and no Stage-2 semantic dependency has entered
  the Stage-1 model or training data contract;
- the candidate-region handoff is validated as eight coordinates plus one
  quality score, with semantics deliberately absent.

### Failure Routing and Authorized Ablations

- If G1 misses aggregate or per-fit coverage, fix conversion or fallback policy;
  do not lower the agreed 98-percent floors without a new decision.

- If G3 fails mainly for 16–32 pixel instances because no useful P3 point exists,
  evaluate P2 as the first architecture ablation. Do not add P2 preemptively.

- If G5 rejects exact-IoU score targets, use the approved corner-quality proxy;
  do not replace direct corner regression with GWD.

- If G6 learns corner coordinates but G7 stalls specifically at high polygon
  IoU, run the predeclared GWD auxiliary-loss ablation against the unchanged
  direct-corner baseline.

- If G7 loses recall only after NMS, isolate threshold or suppression behavior
  using the stored pre-NMS candidates before changing training.

- If G7 fails only on long objects because area scale chooses insufficient
  context, evaluate maximum extent as the registered scale-compatibility
  ablation.

- Any ablation changes one declared factor, reruns its governing tests, and uses
  the same frozen benchmark. Successful ablations update this specification
  before becoming the production baseline.
