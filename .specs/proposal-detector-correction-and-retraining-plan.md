# Proposal Detector Correction and Retraining Plan

## Active scope: proposal detector stopping point

This plan is a correction/retraining sequence within the original project's
detector implementation and training phases. Its phase numbers do not replace
`docs/phase2.md` or `docs/phase3.md`. See the
[project roadmap](../projects/visual-inference/docs/proposal_detector_roadmap.md).

The owner-approved current milestone ends after controlled retraining and
acceptance of HBB/quad proposal outputs. The following scope rules take
precedence over downstream/full-release requirements retained below:

- Phase 2.1 compares proposals using corrected data and matched short runs.
  The owner-approved local HBB/quad loader gate passed on 2026-09-05 for both
  train and validation indexes in parent and spawned-worker processes. Remote
  archive verification and clean-worker staging are deferred until the first
  cloud run; local source symlinks do not establish archive portability.
  Include the P2/min-4 ablation justified by the 16px source-object requirement.
  Quad P2 needs proposal-level justification; no SigLIP experiment is required.
- Phase 2.2 currently fixes K and the common decoded proposal record, source
  coordinates and transform inversion. SigLIP crop rate, crop generation and
  semantic crop audits are deferred.
- Phase 2.3–2.4 use proposal recall, duplicates, background false proposals,
  size/domain slices and inference/postprocessing cost. Downstream crop metrics
  and SigLIP evaluation, including the proposed 1–2 point tolerance, are deferred.
- Phase 2 selects a working architecture for loss validation and retraining,
  retaining a control. This is not final HBB/quad selection for the semantic
  system. Phase 3 selection uses proposal-level budgets and approved thresholds.
- Phase 2.5 retains early export parity and resource profiling. Physical RPi
  production acceptance belongs to a later release milestone; record measured
  evidence and unknowns without treating desktop measurements as RPi evidence.
- Phase 4 retains full retraining, proposal validation, visual review, export
  parity and reproducible artifact packaging. SigLIP-dependent parts of 4.7 and
  physical-device promotion/canary requirements in 4.8 are deferred. Stop after
  the owner reviews the trained proposal outputs, before semantic integration.

Completion of this milestone does not assert completion of the eventual full
system release criteria below. Preserve historical benchmark data and record
new experiment contracts/results separately. Continue plan–execute–verify with
owner review at the agreed step boundaries.

## Problem Statement

The current class-agnostic proposal detector has learned useful objectness and
localization, but the latest visual and metric audits exposed limitations that
cannot be solved by simply training for more epochs:

- supervision differs materially by source dataset;
- large regions that are known background for this product are sometimes
  neutral rather than negative, so they can produce high-scoring proposals
  without a training penalty;
- ego-vehicle body regions are especially under-supervised in fisheye data;
- grouped objects and ambiguous construction regions need a consistent policy;
- the WoodScape validation split currently has no usable proposal labels;
- the nominal eight-pixel object-size cutoff is measured after resizing to the
  model input, not in source-image pixels;
- small local batches do not reproduce the configured domain proportions;
- some COCO instance identity and crowd semantics may have been lost during
  source conversion and deduplication;
- the HBB training path applies the configured photometric augmentations, while
  the quad training path currently applies only horizontal flip, scale, and
  translation even though both paths receive the same augmentation config;
- the quadrilateral head and polygon post-processing may be more complex than
  the downstream rectangular SigLIP crop contract requires;
- loss weights are hand-selected even though their effective contributions and
  gradient magnitudes differ by orders of magnitude.

The production manifests are large, while existing audit reports cover only
small or stale subsets. The next model therefore needs a reproducible correction
program rather than isolated configuration edits. That program must establish a
single object/supervision contract, regenerate and validate the data, compare the
right network output geometry, calibrate the loss against the proposal-stage
goal, and only then perform a controlled retraining.

The work is organized into **four phases**, despite the request saying three,
because four separate workstreams were named and each needs its own stop/go gate:

1. dataset corrections and regeneration;
2. neural-network architecture updates;
3. loss-function adjustment;
4. retraining and release evaluation.

### Ownership Legend

Every task is marked with one of the following ownership labels:

| Label | Owner | Meaning |
|---|---|---|
| **MANUAL** | You | A product, semantic, cost, or visual-quality decision that should not be delegated blindly. |
| **AI AGENT** | Coding agent | Deterministic implementation, test, report, configuration, or experiment orchestration work that can be delegated after its inputs are fixed. |
| **JOINT** | You + AI agent | The agent prepares evidence or candidates; you review and approve the conclusion. |

The agent should never silently decide what counts as an object, whether a
particular ambiguous category is useful to the product, or whether a downstream
crop is acceptable. Those are product decisions even when the implementation is
automated.

### Current Baseline to Preserve

The current production training data contains approximately 267,800 images:

| Dataset | Train images | Positive regions | Ignore regions | Trusted-background regions |
|---|---:|---:|---:|---:|
| COCO | 118,287 | 1,054,975 | 35 | 0 |
| BDD100K | 70,000 | 1,286,852 | 19 | 7,200,810 |
| nuImages | 67,279 | 1,319,184 | 2 | 17,057,260 |
| WoodScape | 12,234 | 247,195 | 116,382 | 1,982,870 |

The corresponding validation sets contain roughly 34,200 images, but WoodScape
contributes 2,766 images with zero positive, ignore, or trusted-background
labels. It therefore cannot currently measure WoodScape proposal quality.

The completed minimum-eight-pixel run showed a useful separation between
explicit positives and explicit trusted background: median predicted quality was
about 0.486 for positives and 0.006 for trusted background. Weak or unlabeled
regions had a much higher median of about 0.257. This is strong evidence that the
model can learn negative supervision when the dataset provides it, and that the
main immediate failure is incomplete or inconsistent supervision coverage.

The latest loss decomposition was also highly imbalanced: approximately 96% of
the total scalar loss came from the weighted corner term, while quality was only
a few percent and validity was negligible. Scalar contribution alone is not the
same as gradient influence, but it is sufficient reason to measure shared-trunk
gradient norms before accepting the current weights.

## Solution

Execute the following four phases in order. A later phase may build tooling while
an earlier phase is being reviewed, but no expensive comparison or final training
run may use inputs that have not passed the preceding gate.

### Phase 1 — Dataset Corrections and Regeneration

#### 1.1 Freeze the product-level object contract

**MANUAL** Write and approve a short, explicit definition of what the proposal
stage should return. The recommended contract is:

> A positive is a bounded, visually identifiable physical instance whose crop
> could be usefully classified or described by the downstream SigLIP stage.
> Stuff, surfaces, textures, reflections, shadows, ego-platform bodywork, and
> scene-layout regions are background. Regions that might contain several
> unresolved instances or whose semantics are uncertain are ignored rather than
> forced positive or negative.

**MANUAL** Decide the final policy for every disputed family before regeneration.
The recommended starting matrix is:

| Region/category | Current effective treatment | Recommended treatment | Rationale / required review |
|---|---|---|---|
| Individual vehicles and pedestrians | Positive when annotated | **Positive** | Core promptable instances. |
| Road / drivable surface | Trusted negative in selected sources | **Trusted negative** | A surface, not an independent proposal target. |
| Sky | Trusted negative only where explicitly mapped | **Trusted negative** | Prevents high-scoring texture and horizon proposals. |
| Vegetation / green strips | Mixed or neutral | **Trusted negative** | Treat as scene stuff unless the product must propose individual plants or trees. Confirm this product choice manually. |
| Ego/self-driving vehicle body | Usually discarded or neutral | **Trusted negative** | Directly targets persistent WoodScape false positives on hood/bodywork. |
| Buildings and walls | Commonly neutral | **Trusted negative** for this proposal product | Suppresses facade and wall proposals. If whole buildings must be discoverable, keep them positive instead; this is a product decision. |
| Static construction regions | Often ignore | **Trusted negative** after visual mapping review | Separate scene construction/barriers from actual construction vehicles and machines. |
| Construction vehicles/machines | Dataset-dependent | **Positive** | Independently promptable physical instances. |
| Grouped vehicles/pedestrians | Ignore in WoodScape | **Ignore**, unless individual instances can be recovered | A group polygon is not a clean instance target and should not become background. |
| License plates | Inconsistent/nested | **Ignore** when contained inside a vehicle, unless plate detection is an explicit downstream requirement | Avoid duplicate nested proposals and evaluation ambiguity. |
| Lane markings and suitable road markings | Mixed | **Trusted negative** | Suppress painted-surface proposals, subject to class mapping review. |
| Curb / free-space labels | Mixed | **Trusted negative** when the polygon is reliable | Scene layout rather than an object. |
| Void / soiling / uncertain masks | Mixed | **Ignore** | Uncertain pixels must not become confident negative supervision. |

**JOINT** Have an agent generate a stratified HTML audit bundle with at least 100
examples per disputed category and source dataset. It should overlay the source
annotation, proposed normalized category, proposed supervision state, source
category name, and source annotation identity. You review the bundle and record
accept/reject/exception decisions. The agent then turns only the approved matrix
into conversion policy.

**Acceptance criteria**

- Every source category maps to positive, trusted negative, ignore, or explicit
  drop-with-reason.
- The same semantic category has the same meaning across datasets unless a
  documented annotation-quality exception requires otherwise.
- Nested objects, group annotations, crowd regions, and uncertain masks have
  explicit policies.
- No converter-specific hidden rule can override the central contract.

#### 1.2 Repair source identity and crowd handling

**AI AGENT** Preserve these provenance fields through normalization and manifest
generation: source dataset, source split, source image identity, source annotation
identity, original category, canonical category, original crowd/group flag,
geometry conversion method, supervision state, and any exclusion reason.

**AI AGENT** Rebuild the COCO conversion from official annotation identities
rather than deduplicating only by class and exact enclosing rectangle. Merge
multi-polygon segments belonging to one official annotation into one physical
instance; do not merge different annotations merely because their enclosing
boxes coincide. Preserve `iscrowd` as ignore unless the approved contract says
otherwise.

**AI AGENT** Add reports that quantify, per dataset and category:

- source annotations read;
- unique source annotation identities retained;
- multi-part instances merged;
- exact geometric duplicates removed and why;
- crowd/group regions mapped to ignore;
- annotations dropped, with mutually exclusive reason counts;
- geometry conversion tier used: source quad, fitted rectangle, fallback HBB,
  or invalid/ignored.

**JOINT** Review a random sample plus all high-multiplicity deduplication groups.
The current conversion removed roughly one million COCO rectangle/polygon
representations, so a count-only assertion is insufficient.

**Acceptance criteria**

- One official physical-instance annotation produces one proposal target.
- Crowd/group semantics survive conversion.
- Every removal is traceable to source identity and a declared rule.
- Regeneration is deterministic: identical inputs and policy produce identical
  manifest hashes.

#### 1.3 Make supervision state a generated-data property

**AI AGENT** Move contained-component and category-state rules into the common
manifest-generation layer. The HBB and quadrilateral loaders must consume the
same already-decided positive/ignore/trusted state. A loader must not reinterpret
an annotation based on its output geometry.

**AI AGENT** Generate trusted-negative polygons for the approved scene classes,
including ego vehicle, buildings/walls, road, sky, vegetation, and applicable
markings. Keep uncertain areas as ignore masks. Resolve overlaps with a single
documented precedence rule, recommended as:

1. valid positive instance;
2. ignore/ambiguous region;
3. trusted negative;
4. unlabeled/weak region.

This prevents a large background polygon from erasing a valid object and
prevents an ambiguous group from becoming a negative.

**AI AGENT** Add spatial integrity checks for invalid polygons, self-intersection,
out-of-frame coordinates, zero area, duplicate vertices, and impossible overlap
states. Repairs must be logged; unrepairable annotations become ignore rather
than silent negatives.

**Acceptance criteria**

- HBB and quad dataset readers report identical state counts before geometry-
  specific target encoding.
- No positive pixel is simultaneously supervised as trusted negative.
- Ignore wins over negative for ambiguity.
- Every trusted-negative category appears in per-source count and area reports.

#### 1.4 Correct WoodScape training and validation supervision

**AI AGENT** Convert ego-vehicle body polygons into trusted negatives after
validating their spatial coverage. The current source contains on the order of
18,000 such polygons, so this change should materially affect the fisheye domain.

**JOINT** Split WoodScape `construction` examples into visually static regions
and actual physical machinery/vehicles. The agent can cluster and prepare the
review set; you approve the mappings. Ambiguous instances remain ignore.

**AI AGENT** Preserve grouped vehicle and grouped pedestrian regions as ignore
unless individual instances can be reconstructed from source annotations.

**AI AGENT** Build a labeled, sequence-safe WoodScape validation partition.
Frames from the same sequence or near-duplicate capture must not cross training
and validation boundaries. If only a test split lacks labels, create a held-out
validation split from labeled training sequences and reserve the unlabeled split
for qualitative inference only.

**Acceptance criteria**

- WoodScape validation has nonzero positive and background counts.
- Validation images are disjoint from training at sequence level.
- A visual audit confirms ego-body negative masks do not cover external objects.
- Per-domain metrics can be calculated for WoodScape rather than reported as
  empty or silently excluded.

#### 1.5 Reassess small-object policy in model coordinates

**MANUAL** Define the smallest source object that matters at expected camera
resolutions, and the minimum useful downstream crop. The current threshold of
eight pixels is applied after letterboxing to a 384-pixel model input. On a
1,280-pixel-wide source, eight model pixels correspond to roughly 27 source
pixels before any aspect-ratio padding effects. It is therefore not an
eight-source-pixel detector.

**AI AGENT** Produce retention and recall-opportunity histograms by:

- source-pixel short side;
- resized/model-pixel short side;
- object area and aspect ratio;
- dataset and canonical category;
- FPN assignment level;
- distance band where available.

**JOINT** Choose one of these coherent policies based on the product requirement:

| Policy | Benefit | Cost / limitation |
|---|---|---|
| Keep 384 input and P3 minimum stride | Cheapest and closest to the current model | Very small distant objects remain below representable resolution. |
| Add a P2 stride-4 feature level | Better localization of small objects at the same input size | More memory, compute, dense candidates, and potential false positives. |
| Increase model input resolution | Preserves more source detail | Backbone cost grows approximately with image area and may violate RPi latency. |
| Tile or use an ROI stage | Can recover tiny source objects | Substantially changes runtime and proposal merging; likely unsuitable for a simple always-on RPi path. |

Do not lower the annotation threshold further unless the selected architecture
can represent and evaluate those targets. Retaining impossible positives creates
assignment noise, not useful supervision.

#### 1.6 Fix domain sampling for small local batches

**AI AGENT** Replace per-microbatch rounded domain quotas with residual or
epoch-level quota accounting. With a microbatch of eight, current rounding
produces approximately 50% COCO, 15% nuImages, 10% BDD, and 25% WoodScape rather
than the configured 50/18/12/20 mix. Gradient accumulation changes effective
batch size but does not repair this source skew.

**AI AGENT** Log intended and observed sample counts per domain for each epoch
and for each optimizer-step window. Add deterministic sampling tests for batch
sizes used locally and in cloud training.

**MANUAL** Approve the target domain mixture after seeing dataset size,
annotation-density, and failure-rate summaries. Dataset-size proportionality is
not automatically optimal; WoodScape may deserve intentional oversampling to
address fisheye failures.

#### 1.7 Unify augmentation across HBB and quad training

**AI AGENT** Replace the separate effective augmentation behaviors with one
shared augmentation contract used by both HBB and quad datasets. The current
quad path uses horizontal flip, scale jitter, and translation, but silently
ignores configured color jitter, blur, JPEG degradation, and noise. The HBB path
uses all of those operations. This divergence makes architecture comparisons
unfair and makes the recorded quad run configuration misleading.

The shared training pipeline must initially contain:

- horizontal flip with the configured probability;
- aspect-preserving letterbox plus random scale in the configured range;
- horizontal and vertical translation within the configured fraction;
- brightness, contrast, and saturation jitter;
- bounded Gaussian blur;
- bounded JPEG recompression;
- bounded additive image noise;
- common padding, normalization, valid-mask, clipping, and visibility behavior.

Rotation, perspective distortion, Mosaic, MixUp, CutMix, and synthetic object
insertion remain excluded from the initial correction. They materially change
geometry, scene composition, or label meaning and require separate controlled
ablations rather than being added under the parity fix.

**AI AGENT** Centralize stochastic image operations and random-parameter
sampling. Geometry-specific code may transform HBB coordinates or quad vertices,
but it must consume the same sampled flip, scale, translation, and photometric
parameters. With the same source image, seed, and epoch, the two paths must
produce:

- pixel-identical augmented image tensors;
- identical valid masks and affine transform metadata;
- equivalent transformed geometry when an HBB is represented as its four
  rectangular corners;
- equivalent visibility and positive/ignore/trusted-state decisions where the
  geometry itself is equivalent.

**AI AGENT** Keep augmentation deterministic per image and epoch, while changing
the deterministic seed between training epochs. Ensure persistent data-loader
workers observe the updated epoch. Exact-resume behavior must reproduce the same
augmentation sequence after restoration.

**AI AGENT** Make validation, visualization, NMS sweeps, and full evaluation use
deterministic letterboxing only. They must not apply stochastic photometric or
geometric training augmentation. Tiny-overfit mode may disable augmentation when
the purpose is a pure learnability check, but that behavior must be explicit in
the run contract.

**AI AGENT** Validate the augmentation configuration at startup and fail on
unknown or unsupported fields. Log the effective augmentation policy—not just
the requested config—to the run contract and W&B. Add per-epoch counters or a
bounded debug report showing how often every stochastic operation was selected,
so a configured but inactive augmentation cannot remain unnoticed.

**JOINT** Generate a before/after audit grid for every dataset, including
day/night, fisheye, tiny-object, boundary-object, and trusted-background samples.
You approve that augmentations remain physically plausible, preserve target
meaning, and do not systematically erase the small objects the new dataset is
intended to retain.

**Acceptance criteria**

- Every configured augmentation field is either applied by both HBB and quad
  training or rejected explicitly.
- Same-image/same-seed parity holds for image tensors, valid masks, transforms,
  and equivalent geometry/state decisions.
- Augmentation varies between epochs and is reproducible across worker counts
  and exact-contract resume.
- Validation and evaluation are deterministic and augmentation-free.
- The visual audit shows no unacceptable corruption or systematic loss of
  important tiny/thin instances.

#### 1.8 Run a full, streaming, hash-bound production audit

**AI AGENT** Replace small bounded audit reports with a streaming audit over every
generated production record. The report must be bound to the exact raw-source
versions, conversion-policy version, generated-manifest hash, and split hash.

The audit must report at least:

- image counts and unreadable images;
- positive, ignore, trusted-negative, weak, and dropped region counts;
- pixel/area coverage of each state;
- category and source distributions;
- geometry tier and conversion tightness distributions;
- source- and model-coordinate size distributions;
- positive/negative/ignore overlap violations;
- duplicate images within and across splits;
- sequence leakage and source identity collisions;
- objects with no valid FPN assignment;
- sampled overlays for every category/state/dataset combination.

**JOINT** Review all hard failures and the stratified overlay bundle. The AI agent
can flag anomalies and propose mappings, but a person should sign off on semantic
correctness.

#### 1.9 Regenerate and publish an immutable dataset version

**AI AGENT** Regenerate filtered annotations, normalized detections, proposal
manifests, validation manifests, and training indexes from the approved policy.
Write to a new immutable dataset identifier; never overwrite the dataset used by
the previous model. Record source hashes, policy version, tool commit, counts,
split identities, and final artifact hashes in the dataset contract.

**MANUAL** Approve the identifier and authorize upload/publication to the training
bucket. The identifier should describe the data contract rather than a model
hyperparameter; a form such as `phase3-production-bg-policy-v2-YYYY-MM-DD` is
preferable to reusing the earlier `min8` name.

**AI AGENT** Verify a clean-machine round trip: download by identifier, verify all
hashes, create indexes, load samples through both HBB and quad readers, and run a
short batch without accessing unstaged local data.

#### Phase 1 deliverables

- Approved object and supervision matrix.
- Versioned source-to-canonical category map.
- Provenance-preserving normalized annotations.
- Labeled, leakage-safe validation sets for all production domains.
- Immutable regenerated dataset with cryptographic contract.
- Full production audit report and stratified visual review bundle.
- Corrected domain sampler and observed-mixture report.
- Shared, parity-tested HBB/quad training augmentation with an effective-policy
  report and augmentation audit grid.

#### Phase 1 stop/go gate

Do not start architecture comparison until all of the following are true:

- all source categories are accounted for;
- WoodScape validation is measurable;
- COCO identity/crowd audit passes;
- HBB/quad augmentation parity and deterministic-validation tests pass;
- no overlap or split-leakage hard errors remain;
- the chosen small-object policy is representable by the candidate architecture;
- you manually approve the visual audit;
- the clean-machine dataset staging and loader smoke test passes.

### Phase 2 — Neural-Network Architecture Updates

The main decision is not “quads are always better” versus “boxes are always
better.” It is whether quadrilateral geometry creates enough downstream value to
justify its harder regression target and polygon post-processing on the target
low-power platform.

#### 2.1 Define a matched architecture comparison

**AI AGENT** Implement and retain these candidates behind the same backbone,
training-data contract, evaluation harness, and proposal budget:

| Candidate | Geometry | Pyramid | Purpose |
|---|---|---|---|
| A | HBB: objectness/quality plus left-top-right-bottom | P3–P5 | Primary simple deployment candidate. |
| B | HBB | P2–P5 | Small-object candidate if Phase 1 shows P3 cannot represent required targets. |
| C | Quadrilateral: quality plus four corners | P3–P5 | Corrected control matching the trained family. |
| D, optional | Quadrilateral | P2–P5 | Run only if both small-object value and quad-specific crop value are demonstrated. |

Do not run every candidate by default. If the manual small-object requirement is
satisfied by P3, omit P2 candidates. If an early crop audit shows no material
benefit from quads, do not spend a full run on quad P2.

**AI AGENT** Initialize all candidates from the same shared backbone/FPN weights
for a fair comparison. Reinitialize geometry-specific prediction layers. Do not
resume optimizer, scheduler, or EMA state when changing the head, dataset
contract, or loss.

#### 2.2 Standardize the output and crop contract

**MANUAL** Fix the production proposal budget `K` and the maximum downstream
SigLIP crop rate the RPi can afford. Recall without a proposal budget is not the
product metric: hundreds of overlapping boxes merely transfer cost downstream.

**AI AGENT** Give both heads a common post-decoding record containing score,
geometry, source-image transform, level, and stable proposal identity. Ensure all
coordinates invert letterboxing correctly.

**JOINT** Define crop generation for SigLIP:

- rectangular crop from the HBB directly;
- rectangular enclosing crop, optional perspective warp, or masked crop from a
  quad;
- fixed context margin;
- clipping and padding behavior at image boundaries;
- minimum crop resolution;
- aspect-ratio handling and square input preparation.

The agent should generate side-by-side crop audits. You decide whether tight
quads preserve meaning better enough to matter.

#### 2.3 Use geometry-appropriate decoding and suppression

**AI AGENT** Implement deterministic decoding, clipping, invalid-geometry
rejection, score ranking, pre-NMS top-K, NMS, and final top-K for each geometry.
HBB uses optimized rectangle NMS. Quad uses exact or verified approximate polygon
NMS, with exact evaluation retained even if deployment uses an approximation.

**AI AGENT** Treat NMS IoU and score threshold as post-training calibration
parameters, not training-loss weights. The previous sweep suggested higher NMS
thresholds improved AR@100 but retained many overlapping proposals. Repeat the
sweep on the corrected validation data and evaluate the full Pareto curve:
recall, duplicate rate, proposals per image, downstream crops, and latency.

**AI AGENT** Add explicit duplicate metrics, including the fraction of top-K
proposals matched to an already-covered ground-truth object and pairwise overlap
statistics among final proposals. This quantifies the visual “many rectangles on
top of each other” problem.

#### 2.4 Measure proposal utility, not geometry in isolation

**AI AGENT** Report per-domain and aggregate:

- AR@10, AR@50, and AR@100 across the agreed IoU range;
- recall at K=10/50/100 for IoU 0.50 and 0.75;
- small/medium/large and thin-object recall;
- source-coordinate size-band recall;
- object crop coverage;
- fraction of crop pixels belonging to object versus background;
- proposal duplication and background false-proposal budgets;
- ego-body, sky, road, vegetation, building/wall, and license-plate false-
  proposal rates;
- decode, NMS, and crop-generation latency separately from neural inference.

**JOINT** Run a downstream SigLIP proxy or the real next stage on a fixed labeled
crop set. HBB is acceptable if it remains within roughly one to two percentage
points of the useful top-K recall of quads, produces equivalent downstream
classification/retrieval quality, and materially improves deployment simplicity
or latency. This tolerance is a proposed gate, not a universal constant; you
must approve it.

#### 2.5 Establish deployment feasibility early

**AI AGENT** Export every serious candidate through the intended deployment path
and test numerical parity before full training. Measure:

- parameter count and model artifact size;
- multiply-accumulate operations at the chosen input resolution;
- peak training and inference memory;
- desktop GPU throughput for experiment planning;
- target RPi latency, memory, temperature/throttling behavior, and power if
  hardware is available;
- post-processing latency at realistic pre- and post-NMS candidate counts.

**MANUAL** Run or authorize the physical-device benchmark if the agent cannot
access the target RPi. Desktop timing is not a substitute for the release gate.

#### Phase 2 deliverables

- Reproducible HBB deployment candidate and retained quad control.
- Optional P2 ablation only if justified by the small-object requirement.
- Common proposal/crop contract and visual crop audit.
- Correctness and parity tests for decode, NMS, coordinate inversion, and export.
- Matched architecture comparison report, including target-device evidence.

#### Phase 2 stop/go gate

Choose one primary architecture before tuning loss weights. Selection requires:

- no export or coordinate-parity failures;
- acceptable target-device inference and post-processing cost;
- the approved top-K recall and downstream crop-quality threshold;
- acceptable false-proposal and duplicate budgets in every critical domain;
- a documented reason for choosing HBB or quad;
- one retained control architecture for detecting regressions.

### Phase 3 — Loss-Function Adjustment

Loss work starts only after data semantics and output geometry are stable.
Otherwise a loss search would compensate for label defects or compare different
tasks.

#### 3.1 Establish a corrected-data baseline before changing weights

**AI AGENT** Train a short, fixed-budget baseline using the selected architecture
and the closest compatible current loss. This isolates the benefit of corrected
supervision from the benefit of loss changes.

For the quad control, the current conceptual objective is:

```text
total = quality
      + 2.0 * corner
      + 0.0 * gwd
      + 0.05 * validity
```

Quality itself includes positive, trusted-negative, and weak-region terms. The
current weak-negative coefficient is zero, so ordinary unlabeled regions do not
train the quality head. Trusted-background examples do train it and are already
well separated in the completed model.

**AI AGENT** Log raw and weighted components separately, by domain and state.
Never infer balance only from the final summed scalar.

#### 3.2 Correct reduction and normalization before tuning coefficients

**AI AGENT** Make every component invariant, as far as practical, to batch size,
positive count, polygon sample count, and image annotation density:

- normalize positive localization by the exact sum of assignment/sample weights;
- reduce positive, trusted-negative, and weak-region quality per image before
  averaging across images, so dense datasets do not dominate by pixel count;
- use explicit safe denominators and log empty-state batches;
- keep ignore regions completely outside positive and negative reductions;
- report the effective number and total weight of samples entering each term;
- preserve deterministic behavior under gradient accumulation.

**AI AGENT** Add scale checks showing that duplicating an identical sample or
changing microbatch partitioning does not materially change the normalized loss
or one-step parameter update.

#### 3.3 Measure gradient influence on shared features

**AI AGENT** Instrument gradient norms and cosine similarity from each loss family
at the shared FPN output and representative backbone layer. Collect these on a
fixed calibration subset after warm-up and at later checkpoints.

Report:

- unweighted and weighted scalar loss;
- gradient norm per loss family;
- ratio to the total shared-feature gradient;
- gradient cosine similarity/conflict between quality and localization;
- values by domain and object-size band;
- variance across calibration batches.

The current scalar breakdown—corner contributing roughly 96%—does not prove that
corner gradients dominate, but the new measurements will.

#### 3.4 Calibrate initial weights automatically, then search narrowly

**AI AGENT** Use the measured median gradient norms to propose one-time initial
coefficients that put quality and localization in a controlled ratio at shared
features. Freeze the coefficients for the main experiment unless a deliberately
tested adaptive method is introduced. Avoid continuously changing weights based
on noisy per-batch gradients in the first production iteration.

Recommended procedure:

1. warm the selected head for a short fixed number of optimizer steps;
2. measure robust median gradient norms on a fixed stratified subset;
3. choose a target quality-to-localization gradient ratio;
4. scale coefficients once and record the resulting values in the experiment
   contract;
5. rerun the calibration subset to verify achieved ratios;
6. compare against the unchanged-weight baseline.

**MANUAL** Approve the target ratio based on product failures. If background
false proposals are the dominant issue, quality/ranking deserves more influence;
if correct objects are found but geometry is unusable, localization deserves
more.

**AI AGENT** Run a small, factorial or successive-halving search rather than a
large blind sweep. A reasonable initial search space is:

| Parameter | Initial candidates | Purpose |
|---|---|---|
| Weak-negative weight | 0, 0.01, 0.03, 0.05 | Penalize unlabeled background cautiously without assuming it is fully annotated. |
| Quality focal exponent | 1, 2 | Control focus on hard score examples. |
| Quality/localization gradient target | Baseline plus two nearby ratios | Test ranking versus geometry balance. |
| Trusted-negative weight | 1.0 plus one lower candidate only if negatives dominate | Trusted regions are explicit and should normally retain full weight. |
| Validity weight for quads | Current value and zero, after normalization | Determine whether it has measurable effect; current contribution is negligible. |
| GWD | Off by default | Add only if direct geometry improves but high-IoU recall stalls. |

The observed unweighted weak loss suggests coefficients of 0.01, 0.03, and 0.05
would initially add only about 0.00032, 0.00097, and 0.00162 to the scalar loss.
Those are sensible cautious candidates, but gradient measurements and validation
outcomes—not scalar arithmetic alone—must decide.

#### 3.5 Use goal-aligned selection metrics

**MANUAL** Define the primary selection objective before running the search. The
recommended form is constrained optimization:

> Maximize useful recall at the production proposal budget and downstream SigLIP
> utility, subject to per-domain false-proposal, duplicate, latency, and crop-cost
> limits.

**AI AGENT** Evaluate every candidate on the same fixed validation data and report
confidence intervals or seed variation where feasible. Do not select by training
loss alone. Include explicit slices for ego vehicle, sky, road, vegetation,
facades/walls, tiny objects, thin objects, and crowded scenes.

#### 3.6 Add targeted hard-negative mining only after the clean baseline

**AI AGENT** Run the best baseline over a held-out mining pool, collect high-score
unmatched proposals, and categorize them by overlap with trusted background and
source semantic regions. Deduplicate near-identical crops and cap contributions
per source image/domain.

**JOINT** Review a stratified mining sample. A false positive is promoted to a
hard negative only when a human-approved rule or reliable source annotation
establishes that it is truly background. Potential unlabeled objects remain weak
or ignore.

**AI AGENT** Add approved hard negatives as a versioned supplemental manifest,
not by modifying the original source annotations in place. Compare one targeted
mining round against the non-mined baseline. Avoid repeated self-training loops
until the first round demonstrates value.

#### 3.7 Calibrate scores and NMS after training

**AI AGENT** Sweep score thresholds and NMS thresholds on validation outputs,
including per-domain reliability curves. Proposal quality is a ranking score, not
necessarily a calibrated probability. Choose thresholds against the fixed top-K
and false-proposal budget; do not treat a visually convenient threshold such as
0.4 as universal.

#### Phase 3 deliverables

- Corrected, batch-invariant loss reductions.
- Component and shared-gradient diagnostics.
- Reproducible one-time coefficient calibration report.
- Narrow loss ablation results against a corrected-data baseline.
- Optional reviewed hard-negative supplement.
- Calibrated score/NMS operating points.

#### Phase 3 stop/go gate

Proceed to full retraining only when:

- loss normalization tests pass across microbatch partitions;
- no component is silently inactive unless intentionally disabled;
- chosen coefficients have measured, documented gradient behavior;
- the selected loss improves the predeclared proposal objective on validation,
  not merely training loss;
- critical-domain false proposals do not regress;
- the exact configuration and calibration evidence are versioned.

### Phase 4 — Retraining, Evaluation, and Release

#### 4.1 Freeze the training contract

**AI AGENT** Produce one immutable run contract containing:

- source commit and uncommitted-diff status;
- dataset identifier and manifest/index hashes;
- architecture and output-geometry choice;
- input size, pyramid levels, augmentation, and size policy;
- batch size, accumulation, observed domain mixture, workers, and seeds;
- optimizer, scheduler, learning rates, warm-up, weight decay, clipping, and EMA;
- all normalized loss definitions and weights;
- validation cadence, proposal budgets, score/NMS calibration protocol;
- checkpoint cadence, local retention, S3 destination, and W&B project/run
  identity;
- expected hardware and software/container identity.

**MANUAL** Approve cloud cost limits, maximum runtime, validation cadence, and
early-stop rules. Validation every five epochs is reasonable for the current
long validation path, but final selection needs validation at the end and around
any suspected optimum.

#### 4.2 Use weights-only warm starts correctly

Because the dataset policy, potentially the output head, and the loss will
change, do not continue the old optimizer trajectory as if this were the same
run.

**AI AGENT** Apply this policy:

- corrected quad model: initialize compatible backbone/FPN and optionally quad
  head weights from the previous checkpoint, but reset optimizer, scheduler,
  scaler, EMA, epoch, and run identity;
- HBB model: initialize the same compatible backbone/FPN weights and initialize
  the new HBB prediction layers from scratch;
- P2 model: initialize shared levels and initialize new P2 lateral/output layers
  from scratch;
- fair architecture comparison: use the same shared-trunk checkpoint and the
  same seed policy for every candidate;
- checkpoint resume after infrastructure interruption: restore full state only
  when dataset, model, optimizer, scheduler, and loss contract hashes match.

**MANUAL** Decide whether the final champion also needs a from-scratch control.
For this scale of correction, a weights-only warm start is efficient and valid,
but one shorter from-scratch or alternate-seed run is valuable to detect inherited
bias if budget permits.

#### 4.3 Execute escalating preflight tests

**AI AGENT** Run these in order and stop on failure:

1. **Static contract check:** hashes, schema versions, source mappings, export
   compatibility, and no dirty-code ambiguity.
2. **Dataset load smoke:** batches from every domain with positive, ignore,
   trusted, and weak examples; visual target overlays before augmentation and
   after transform inversion.
3. **Tiny overfit:** a small mixed-domain subset must substantially reduce both
   quality and localization errors and recover its targets after decoding.
4. **Single-optimizer-step equivalence:** compare equivalent effective batches
   formed with different accumulation partitions.
5. **Short GPU smoke:** exercise mixed precision, EMA, validation for both states,
   checkpoint upload, W&B streaming, and clean shutdown.
6. **Safe-resume smoke:** interrupt after a checkpoint, resume the exact contract,
   and verify next-step continuity, sampler position policy, scheduler state,
   optimizer state, EMA state, and artifact hashes.
7. **Contract-mismatch negative test:** prove that an incompatible full-state
   resume is rejected with an actionable message while an explicit weights-only
   initialization remains available.

**JOINT** Review smoke-test overlays, component curves, proposal counts, GPU
memory, data throughput, and validation output before authorizing the full run.

#### 4.4 Run the controlled architecture/loss experiments

**AI AGENT** Use short budgets and the same image/optimizer-step exposure for
candidate selection. Do not compare runs only by epoch if samplers or dataset
sizes differ. Log both images seen and optimizer steps.

Recommended experiment sequence:

1. previous-compatible architecture on corrected data with baseline loss;
2. selected HBB versus retained quad control under matched initialization;
3. P2 ablation only if required;
4. normalized/calibrated loss candidates on the chosen architecture;
5. one approved hard-negative comparison, if Phase 3 justified it;
6. promote only the winning configuration to a full run.

**MANUAL** Select the winner using the predeclared gate, not the lowest training
loss or a single attractive aggregate metric.

#### 4.5 Launch and monitor the full retraining run

**AI AGENT** Configure local checkpoints, immutable remote checkpoint uploads,
latest/previous checkpoint manifests, integrity hashes, and W&B metrics. Log
timestamps and explicit `state=raw` / `state=ema` validation progress.

At minimum, monitor:

- data and step time, images per second, GPU/CPU utilization, host and GPU memory;
- epoch, microbatch, optimizer step, images seen, and ETA;
- total and component loss, both raw and weighted;
- per-domain batch composition and sample-state counts;
- learning rate, gradient norm, scaler state, and skipped steps;
- raw and EMA validation metrics;
- proposals per image, duplicate rate, and background-slice false proposals;
- checkpoint upload success and remote hash verification.

**MANUAL** Intervene only on predefined conditions: non-finite loss, persistent
OOM, corrupt input, repeated checkpoint failure, validation collapse, material
false-proposal regression, or cost-limit breach. A noisy individual batch loss
or one allocator warning followed by successful progress is not automatically a
reason to stop, but memory growth must be investigated if sustained.

#### 4.6 Apply stopping and model-selection rules

**MANUAL** Approve the exact thresholds before launch. Recommended rules:

- choose by the constrained validation objective, normally EMA if it consistently
  outperforms raw weights, but retain both artifacts;
- require improvement larger than validation noise or seed variation;
- stop after a defined number of full validation events without improvement;
- reject a global improvement that violates a critical domain/background budget;
- never select from training loss alone;
- if raw consistently beats EMA, revisit EMA decay rather than automatically
  discarding raw weights.

**AI AGENT** Produce a checkpoint-comparison table at every validation event and
maintain best-by-primary-metric and best-by-constraint artifacts separately from
the latest resumable checkpoint.

#### 4.7 Perform final validation and downstream audit

**AI AGENT** Evaluate the selected raw and EMA checkpoints over the full,
hash-bound validation set. Report aggregate, per-dataset, category family,
object-size, aspect-ratio, and trusted-background slices. Include confidence
intervals or multiple seeds for the final claimed improvement when practical.

**AI AGENT** Generate a large balanced visual audit: at least 150 images per
validation dataset, sampled across scene, size, and failure strata, with
interactive score threshold, NMS setting, positive/ignore/trusted overlays, and
proposal toggles.

**MANUAL** Review false negatives, duplicate proposals, ego-body proposals,
facades/walls, road/sky/vegetation, tiny distant people/vehicles, grouped scenes,
and crop usefulness for SigLIP. Record a release decision and known limitations.

**JOINT** Run the downstream SigLIP evaluation on the frozen proposal outputs.
This is the decisive check that an HBB simplification or quad improvement helps
the full system rather than only proposal IoU.

#### 4.8 Benchmark and package the production artifact

**AI AGENT** Export the selected model, verify numerical parity, benchmark the
complete proposal pipeline at the chosen operating point, and package:

- model weights and raw/EMA provenance;
- export artifact and runtime requirements;
- preprocessing and coordinate transform contract;
- decoding, NMS, score threshold, top-K, and crop parameters;
- dataset and code hashes;
- validation report, visual audit, and RPi benchmark;
- known failure modes and rollback artifact.

**MANUAL** Run the final physical-device acceptance test and approve promotion.
Keep the previous production model available until the new model passes an
end-to-end canary on representative camera input.

#### Phase 4 deliverables

- Fully reproducible training contract.
- Verified local and remote checkpoint/resume path.
- W&B run with complete raw/EMA, data, loss, and systems telemetry.
- Selected checkpoint plus export artifact.
- Full validation, visual audit, downstream SigLIP comparison, and RPi benchmark.
- Release decision, model card, known limitations, and rollback path.

#### Phase 4 completion gate

The retraining program is complete only when:

- exact data/code/config/artifact hashes are recorded;
- final validation passes the predeclared recall, false-proposal, duplicate, and
  per-domain constraints;
- downstream SigLIP utility is no worse than the approved tolerance;
- target-device latency, memory, and thermal behavior are acceptable;
- checkpoint restoration and export parity are verified;
- a human has approved the balanced visual audit;
- the previous artifact remains recoverable for rollback.

### Recommended Execution Order and Human Effort

The phases are sequential at their gates, but preparatory engineering can overlap:

| Order | Work package | Primary owner | Human attention | Dependency |
|---:|---|---|---|---|
| 1 | Object/supervision policy and disputed-category review | **JOINT** | High | None |
| 2 | Provenance, COCO identity, common state generation, WoodScape split | **AI AGENT** | Medium review | Approved policy |
| 3 | Full audit and immutable regeneration | **AI AGENT** | High sign-off | Correct converters |
| 4 | HBB/control implementation and deployment parity tests | **AI AGENT** | Low until crop review | Stable geometry contract |
| 5 | Small-object/P2 decision and crop-contract review | **JOINT** | High | Data size audit |
| 6 | Architecture short runs and RPi feasibility | **JOINT** | Medium/high | Phase 1 gate |
| 7 | Loss normalization and gradient instrumentation | **AI AGENT** | Low | Chosen architecture |
| 8 | Narrow loss search and optional hard-negative review | **JOINT** | Medium | Corrected baseline |
| 9 | Preflight, resume test, and full retraining | **AI AGENT** | Launch approval and exception handling | Phase 3 gate |
| 10 | Visual, downstream, and physical-device release audit | **JOINT** | High | Final checkpoints |

Tasks that are especially suitable for outsourcing to an AI coding agent are
converter refactors, schema/provenance work, deterministic regeneration,
streaming audits, test fixtures, sampler correction, shared augmentation and
parity tests, model-head implementation, decode/export parity, gradient
instrumentation, experiment configuration, checkpoint/resume validation, W&B
dashboards, and report generation.

Tasks that should remain manual are the object contract, ambiguous semantic
adjudication, minimum useful object/crop definition, domain weighting intent,
target-device cost limits, downstream quality tolerances, release thresholds,
visual audits, and final promotion/rollback decisions.

## Implementation Decisions

- The existing class-agnostic quad detector specification remains the historical
  baseline. This plan corrects its data and selection process without erasing the
  original architectural rationale.
- Supervision semantics are centralized in generated manifests and are
  independent of whether the consumer trains HBBs or quads.
- Trusted negative means explicitly annotated, reliable background for this
  product. Weak means unlabeled or incompletely annotated and must not be treated
  as equivalent to trusted background.
- Positive overrides trusted background spatially; ignore overrides negative;
  unknown remains weak.
- HBB and quad training share one augmentation parameter sampler and one
  photometric implementation. Geometry adapters may differ, but identical
  source image, seed, and epoch must yield identical pixels, valid masks, affine
  metadata, and equivalent state decisions. Validation never uses stochastic
  augmentation.
- The initial shared augmentation contract contains flip, scale, translation,
  color jitter, blur, JPEG degradation, and noise. Rotation, perspective,
  Mosaic, MixUp, CutMix, and synthetic insertion require separate ablations.
- HBB is the preferred deployment hypothesis because SigLIP ultimately consumes
  rectangular crops, but it is not selected without a matched empirical test.
- Quad remains the control and can win if its tighter/perspective-aware crops
  materially improve useful recall or downstream performance within the RPi
  budget.
- P2 or higher input resolution is introduced only if the manually defined
  small-object requirement cannot be represented by P3 at 384 input.
- Architecture is selected before loss tuning so coefficients are not tuned for
  a discarded output geometry.
- Loss terms are normalized before weights are searched. Gradient influence on
  shared features is measured; scalar loss magnitudes alone do not determine
  weights.
- Automated loss weighting means one-time, reproducible calibration plus a
  narrow goal-aligned search in the initial iteration, not an uncontrolled
  continuously adaptive training algorithm.
- Dataset/loss/head changes use weights-only initialization and reset optimizer,
  scheduler, scaler, EMA, epoch, and W&B run identity. Exact interrupted runs may
  restore full state only after contract-hash equality.
- Raw and EMA weights are evaluated and reported separately throughout.
- Score and NMS thresholds are validation-calibrated deployment parameters, not
  substitutes for data correction or loss supervision.
- Dataset identifiers are immutable and content-addressed by a contract. Old
  production data and checkpoints are never overwritten.

## Testing Decisions

The highest-value test seam is the complete dataset contract:

```text
small raw-source fixture
  -> normalized canonical annotations with provenance
  -> proposal manifest with positive/ignore/trusted/weak states
  -> streaming index
  -> transformed training sample and decoded visual target
```

This seam should be tested end to end for every dataset family and for both HBB
and quad consumers. It catches policy drift earlier and more reliably than tests
that mock intermediate annotations.

Required tests and validations are:

1. **Source conversion contract tests**
   - official annotation identity and multi-part instance preservation;
   - crowd, group, nested component, and ambiguous-region handling;
   - canonical category and state mapping;
   - deterministic output hashes;
   - geometry repair/fallback reason accounting.

2. **Spatial supervision tests**
   - positive/ignore/trusted precedence;
   - no contradictory pixel supervision;
   - ego-body masks do not erase external positives;
   - invalid polygons are repaired deterministically or ignored;
   - HBB and quad loaders consume identical states.

3. **Augmentation contract tests**
   - same-image/same-seed HBB and quad outputs have pixel-identical image
     tensors, masks, and affine metadata;
   - an HBB and its equivalent four-corner rectangle remain geometrically
     equivalent after flip, scale, translation, clipping, and visibility rules;
   - every configured photometric operation can be forced independently and is
     applied identically to both geometry paths;
   - unknown or unsupported augmentation settings fail configuration validation;
   - epoch changes alter the deterministic augmentation sequence, while fixed
     seed/epoch results reproduce across worker counts and exact resume;
   - validation, visualization, NMS sweep, and evaluation outputs are
     deterministic and contain no stochastic training augmentation;
   - a high-seam paired-dataset fixture exercises source record through final
     transformed HBB and quad samples rather than testing private helper calls.

4. **Split and corpus tests**
   - no image or sequence leakage;
   - no unexpected source identity collisions;
   - all validation domains contain usable labels;
   - full streaming counts reconcile with generated contracts;
   - unreadable images and missing artifacts fail explicitly.

5. **Sampler tests**
   - observed epoch mixture matches the configured quota within one-sample
     rounding tolerance;
   - local microbatch and cloud batch paths preserve the same long-run mixture;
   - fixed seeds reproduce sample order;
   - resume behavior follows the declared sampler-position policy.

6. **Model and decoder tests**
   - output shapes and coordinate ranges for every head/pyramid candidate;
   - letterbox transform round trips;
   - HBB and polygon IoU/NMS against trusted reference cases;
   - invalid and boundary geometry handling;
   - final top-K and duplicate accounting;
   - eager-versus-export numerical parity.

7. **Loss tests**
   - exact analytical cases for positive, trusted, weak, and ignore states;
   - no gradients from ignore regions;
   - normalized loss invariance to duplicated samples and accumulation partition;
   - finite gradients for empty-state and extreme-geometry batches;
   - component gradient instrumentation does not alter the optimizer update when
     diagnostics are disabled;
   - a tiny overfit case improves both ranking and geometry.

8. **Training lifecycle tests**
   - timestamped progress and `state=raw` / `state=ema` validation logs;
   - local checkpoint, remote upload, checksum, and latest/previous manifests;
   - exact-contract full resume continuity;
   - incompatible-contract rejection and explicit weights-only path;
   - W&B reconnect/resume semantics and unique run identity;
   - graceful behavior after upload or validation failure.

9. **Acceptance evaluation**
   - full validation metrics per domain and failure slice;
   - balanced interactive visual audit;
   - downstream SigLIP utility at fixed proposal budgets;
   - target-RPi end-to-end latency, memory, and thermal test;
   - comparison with the previous released checkpoint and rollback rehearsal.

Production cardinality should not be encoded as a brittle fixed regression
constant. Instead, tests assert source reconciliation, state conservation,
determinism, nonempty expected domains, and explicitly reviewed count deltas in
the generated audit contract.

## Out of Scope

- Redefining or training the full open-vocabulary SigLIP classifier beyond the
  minimum downstream evaluation needed to select proposal architecture.
- Introducing instance segmentation as the primary proposal representation.
- Building a generic ontology capable of satisfying every future product.
- Treating every unlabeled pixel as a negative.
- Repeated autonomous hard-negative self-training loops.
- Large unbounded hyperparameter searches.
- Distributed multi-GPU redesign unless single-GPU experiments demonstrate a
  concrete throughput or memory blocker.
- Production deployment rollout beyond artifact packaging, target-device
  acceptance, canary criteria, and rollback preparation.
- Overwriting prior datasets, checkpoints, W&B histories, or release artifacts.

## Further Notes

- The data correction is expected to produce the largest immediate reduction in
  proposals on ego body, sky, road, vegetation, and facades because explicit
  trusted negatives already score far below weak regions in the trained model.
- Buildings, walls, vegetation, and license plates are not inherently positive
  or negative in computer vision. Their status follows the approved product
  contract. Changing that contract later requires a new dataset version.
- “Minimum eight pixels” must always state its coordinate system. Reports and
  configuration names should distinguish source pixels from resized model
  pixels.
- HBB output does not prevent later oriented reasoning. SigLIP can consume an
  enclosing rectangular crop; the empirical question is whether extra
  background harms it enough to justify quads.
- Higher recall values are good only at a fixed proposal budget and acceptable
  false-proposal/duplicate cost. AR@100 and recall@100 should never be read
  without proposals per image, domain slices, and downstream crop cost.
- The proposed HBB acceptance tolerance, gradient-ratio strategy, category
  policies, and RPi constraints are decision points, not hidden defaults. They
  should be approved before implementation or experiment launch.
- The first implementation milestone should end at the Phase 1 gate. Committing
  to a full retraining date before semantic review and regeneration are complete
  would create false precision.
