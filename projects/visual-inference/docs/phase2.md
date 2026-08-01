# Phase 2: class-agnostic student detector

## Objective

The architecture's overall goal is to provide a small, efficient detector for an
embedded open-vocabulary vision system. It must find likely objects on a
low-power target while leaving semantic identification to a later stage that can
compare visual features with arbitrary text labels.

The goal of Phase 2 is narrower: lay out, implement, and validate the detector
architecture before training it. This establishes the backbone, feature pyramid,
prediction head, target assignment, decoding, export contract, and future
semantic-head interface so later phases can add training and open-vocabulary
semantics without redesigning the detector core.

## 1. Purpose and scope

Phase 2 implements the small, class-agnostic proposal generator that will sit at
the front of the embedded open-vocabulary pipeline. Given an image, it predicts
where likely objects are and how large they are. It deliberately does **not**
predict semantic classes yet.

This separation is important. Proposal generation can be trained and evaluated
as a conventional localization problem before adding the more expensive semantic
matching stage. A future semantic stage will produce per-region descriptors
that can be compared with text-derived descriptors. No vision-language model,
semantic loss, or text encoder is part of the current detector.

The implemented phase contains:

- a pretrained MobileNetV4-Conv-Medium feature backbone;
- a three-level, 96-channel lightweight feature pyramid;
- a shared, decoupled class-agnostic detection head;
- point-grid geometry and FCOS-style box encoding;
- an ATSS target assigner;
- a class-agnostic inference decoder and NMS;
- `torch.export` verification, inspection utilities, and tests.

Training and data preparation are maintained separately in Phase 3. DQCO,
region embeddings, SigLIP distillation, quantization calibration, and
ExecuTorch lowering are future work.

## 2. Architecture at a glance

The design combines the mobile-oriented convolutional backbone from
[MobileNetV4](https://arxiv.org/abs/2404.10518), the top-down/lateral structure
of [Feature Pyramid Networks](https://openaccess.thecvf.com/content_cvpr_2017/html/Lin_Feature_Pyramid_Networks_CVPR_2017_paper.html),
the point-relative box representation and centerness concept from
[FCOS](https://openaccess.thecvf.com/content_ICCV_2019/html/Tian_FCOS_Fully_Convolutional_One-Stage_Object_Detection_ICCV_2019_paper.html),
and the adaptive sample-selection rule from
[ATSS](https://openaccess.thecvf.com/content_CVPR_2020/html/Zhang_Bridging_the_Gap_Between_Anchor-Based_and_Anchor-Free_Detection_via_Adaptive_CVPR_2020_paper.html).
It is not an exact reproduction of any one of those detectors; it is a deliberately
smaller composition for low-power deployment.

```text
RGB image: N x 3 x H x W, normally H=W=384
                  │
                  ▼
       MobileNetV4-Conv-Medium
          C3       C4       C5
       stride 8 stride 16 stride 32
          │        │        │
          └────────┴────────┘
                  │
          96-channel Lite FPN
          P3       P4       P5
                  │
        shared decoupled head
          ├── objectness: 1 logit
          └── regression tower
                ├── l,t,r,b: 4 distances
                └── centerness: 1 logit
```

At the primary 384x384 resolution, the tensor contract is:

| Stage | Reduction | Shape | Role |
| --- | ---: | --- | --- |
| C3 | 8 | `N x 80 x 48 x 48` | fine backbone detail |
| C4 | 16 | `N x 160 x 24 x 24` | intermediate context |
| C5 | 32 | `N x 256 x 12 x 12` | coarse contextual feature |
| P3 | 8 | `N x 96 x 48 x 48` | fine fused detector feature |
| P4 | 16 | `N x 96 x 24 x 24` | middle fused detector feature |
| P5 | 32 | `N x 96 x 12 x 12` | coarse detector feature |

At 320x320, the three spatial shapes are 40x40, 20x20, and 10x10.
Therefore the raw detector evaluates 3,024 locations at 384 and 2,100 locations
at 320. Each location emits six values: one objectness logit, four distances, and
one centerness logit.

The random-weight architecture has 7,055,321 parameters. `torchinfo` estimates
approximately 2.51 billion multiply-adds at 384 and 1.75 billion at 320. These
figures help compare architectural revisions; they are not target latency
predictions. Depthwise kernels, memory traffic, thread scheduling, operator
fusion, and INT8 support all affect actual Raspberry Pi performance.

## 3. Backbone

### 3.1 Why MobileNetV4-Conv-Medium

MobileNetV4 was designed around mobile hardware and introduces Universal
Inverted Bottleneck blocks that cover several efficient convolutional block
forms. We use the convolution-only medium variant because the first deployment
target benefits from conventional convolutional operators and a predictable
feature hierarchy. The project creates the pretrained model through
[`timm`](https://github.com/huggingface/pytorch-image-models):

```python
timm.create_model(
    "mobilenetv4_conv_medium.e500_r256_in1k",
    pretrained=True,
    features_only=True,
)
```

`features_only=True` removes the classification pool and classifier and exposes
spatial feature maps. The `r256` tag describes the pretrained recipe; it does not
make the convolutional network incapable of accepting 320x320 or 384x384 inputs.

Backbones do not all map the same output index to the same spatial stride.
Consequently, the wrapper uses `feature_info.reduction()` and
`feature_info.channels()` to locate stride-8, stride-16, and stride-32 features.
This follows the
[`timm` feature-extraction API](https://huggingface.co/docs/timm/v1.0.15/en/feature_extraction),
which explicitly exposes reductions and channel counts for downstream dense
prediction code. If a requested reduction is missing, construction fails with an
error listing the available reductions instead of quietly selecting the wrong
feature.

Implementation: [`student_detector/backbone.py`](../student_detector/backbone.py).

### 3.2 The `timm` C5=960 workaround

The most important backbone integration issue is that the tested `timm` 1.0.28
feature wrapper reports `(80, 160, 960)` at strides `(8, 16, 32)`, while the
desired convolutional stage outputs `(80, 160, 256)`. The 960-channel tensor is
created by a final 1x1 classification-oriented expansion after the useful
256-channel stride-32 stage. Passing 960 channels into the FPN would violate the
architecture contract and add unnecessary 960-to-96 projection work.

The wrapper handles this in a guarded sequence:

1. Construct the full `features_only` model, allowing pretrained weights to load
   normally.
2. Find C3, C4, and the reported C5 from `feature_info` rather than fixed output
   indices.
3. If C3/C4 are 80/160 but C5 is not 256, inspect the module path registered for
   the stride-32 output.
4. Verify that the immediately preceding block really ends with a 256-channel
   convolution.
5. Prune the final expansion so it consumes neither time nor memory during
   inference, then update the feature-output mapping.
6. Verify the final channel tuple is exactly `(80, 160, 256)`; otherwise fail
   loudly and ask the developer to inspect the installed `timm` version.

This workaround necessarily touches `MobileNetV3Features._stage_out_idx`, a
private `timm` implementation detail. It is therefore version-sensitive. The
lockfile records the tested dependency version, runtime checks prevent silent
misconfiguration, and the real-backbone test verifies all three shapes. If a
future `timm` release changes its MobileNet feature wrapper, this code should be
reviewed rather than bypassing `verify_expected_channels`.

## 4. Lite FPN

FPNs make semantically strong deep features available at finer spatial scales
through a top-down path and lateral additions. This is valuable for detection
because objects occupy different numbers of pixels. Our neck retains that core
idea while intentionally omitting P6/P7, PAN paths, BiFPN weights, deformable
convolutions, and attention.

The implemented computation is:

```text
lateral5 = Conv1x1(C5)
lateral4 = Conv1x1(C4) + nearest_resize(lateral5)
lateral3 = Conv1x1(C3) + nearest_resize(lateral4)

P5 = DSConv(lateral5)
P4 = DSConv(lateral4)
P3 = DSConv(lateral3)
```

All lateral projections produce 96 channels. Nearest-neighbour resizing is
deterministic, inexpensive, and maps cleanly to deployment runtimes. Addition is
used instead of concatenation so the pyramid width remains fixed.

`DSConv` is:

```text
3x3 depthwise convolution
1x1 pointwise convolution
BatchNorm
ReLU
```

Depthwise separable convolution was popularized for embedded vision by
[MobileNet](https://arxiv.org/abs/1704.04861): the depthwise operation performs
spatial filtering independently per channel, while the 1x1 pointwise operation
mixes channels. It generally uses fewer parameters and arithmetic operations than
a dense 3x3 convolution at the same width, although real latency must still be
measured on the target kernel library.

BatchNorm was chosen instead of GroupNorm to preserve a conventional
Conv-BN-ReLU fusion path. PyTorch's
[`fuse_modules`](https://docs.pytorch.org/docs/main/generated/torch.ao.quantization.fuse_modules.fuse_modules.html)
documentation lists Conv-BN and Conv-BN-ReLU among supported fusion patterns.
During training, BatchNorm sees the physical per-device batch, not the effective
batch produced by gradient accumulation. If memory forces very small physical
batches, running statistics must be treated deliberately—for example by freezing
backbone BN after warm-up or using synchronized BN during multi-device training.

Implementation: [`student_detector/neck.py`](../student_detector/neck.py).

## 5. Shared detection head

The prediction head is class-agnostic and decoupled:

- the object tower contains one DSConv and one 1x1 objectness predictor;
- the regression tower contains one DSConv, a 1x1 four-distance predictor, and a
  separate 1x1 centerness predictor.

The entire head is shared across P3, P4, and P5. Sharing saves parameters and is
consistent with the parameter-efficient multi-level design used by FCOS. The
three levels differ only through one learned scalar per level and their known
strides `(8, 16, 32)`.

For level `i`, the box output is:

```text
d_i = ReLU(scale_i * raw_distance_i) * stride_i
```

The ReLU ensures the predicted left/top/right/bottom distances are non-negative.
Multiplying by stride inside the head converts them to input-pixel units, so all
levels share one decoder. The learned scales start at 1.0.

The objectness predictor bias is initialized for a prior probability of 0.01:

```text
bias = -log((1 - 0.01) / 0.01) ≈ -4.595
```

This makes initial objectness probabilities small instead of beginning near 0.5,
which is useful when background locations greatly outnumber positives.

`StudentDetector.forward()` returns a fixed `DetectorOutput` named tuple with
three tuples, each ordered P3/P4/P5:

| Field | Per-level shape at 384 |
| --- | --- |
| `objectness` | `N x 1 x {48,24,12} x {48,24,12}` |
| `box_distances` | `N x 4 x {48,24,12} x {48,24,12}` |
| `centerness` | `N x 1 x {48,24,12} x {48,24,12}` |

Implementation: [`student_detector/head.py`](../student_detector/head.py).

## 6. Point and box geometry

Each feature cell corresponds to one input-image point at the cell centre. For a
cell `(column, row)` and stride `s`:

```text
x = (column + 0.5) * s
y = (row    + 0.5) * s
```

For a point `(x, y)` inside a ground-truth box
`(x_min, y_min, x_max, y_max)`, FCOS-style targets are:

```text
l = x - x_min
t = y - y_min
r = x_max - x
b = y_max - y
```

Decoding is the exact inverse:

```text
x_min = x - l      y_min = y - t
x_max = x + r      y_max = y + b
```

The test suite verifies that perfect encoded targets reproduce their original
boxes. The same geometry functions are used by assignment and inference to avoid
subtle train/decode convention mismatches.

Implementation: [`student_detector/geometry.py`](../student_detector/geometry.py).

## 7. ATSS assignment

ATSS argues that the positive/negative sampling rule is a central difference
between anchor-based and anchor-free detectors. We use virtual square priors only
to measure assignment IoU; the network remains point-regression based and never
predicts offsets from those priors.

| Level | Stride | Virtual prior side |
| --- | ---: | ---: |
| P3 | 8 | 32 px |
| P4 | 16 | 64 px |
| P5 | 32 | 128 px |

For every geometrically valid GT box, the assigner:

1. computes squared centre distance from the GT centre to every feature point;
2. selects up to `k=9` nearest points independently from each pyramid level;
3. computes IoU between those candidates' virtual priors and the GT;
4. sets the adaptive threshold to `mean(IoU) + population_std(IoU)`; the code
   uses `unbiased=False`;
5. retains candidates whose IoU reaches the threshold and whose point is inside
   the GT;
6. when one point matches several GTs, assigns it to the GT with the greatest
   virtual-prior IoU;
7. creates `l,t,r,b` targets from the point itself, not from the virtual prior.

The assigner accepts GT boxes with positive area that intersect the image. It
returns flat targets aligned with concatenated P3/P4/P5 locations, original GT
indices, a positive mask, validity information, centerness targets, and any GTs
that the point representation cannot express.

Centerness follows FCOS:

```text
centerness = sqrt(
    min(l, r) / max(l, r)
  * min(t, b) / max(t, b)
)
```

It approaches one near a box centre and zero near its edges. Objectness and
centerness are kept as distinct predictions so training can supervise “is this
an object?” separately from “is this a well-centred localization point?”.

Implementation: [`student_detector/assigner.py`](../student_detector/assigner.py).

### 7.1 The tiny-box fallback correction

The initial plan requested assigning the nearest point whenever a GT received no
positives, specifically to cover extremely small boxes. That rule has a geometric
conflict with the chosen regression parameterization.

If no grid centre lies inside a GT, the nearest point is outside. For example, if
the point lies left of the box, `l = x - x_min < 0`. At least one target distance
must therefore be negative. The head, however, applies ReLU and can emit only
non-negative distances. Training such a target would ask the model to predict a
value it can never produce, and clipping the target to zero would prevent exact
box reconstruction.

The implemented fallback is consequently:

- if ATSS found no positive but an unassigned point exists inside the GT, assign
  the nearest such point;
- if no free inside point exists, report the original GT index in
  `unrepresentable_gt_indices` and do not create an impossible regression target.

Before training, count these cases in the real resized dataset. The available
policy choices are:

1. ignore/filter them for the first detector baseline;
2. add a stride-4 P2 level, increasing compute and activation memory;
3. change to a signed or centre-offset representation, which also changes the
   head, loss, and quantization behavior.

The current code chooses the first policy operationally, while exposing the
affected GT indices so the loss or dataset validator can report them rather than
silently dropping them.

## 8. Inference decoder

Post-processing is deliberately outside `StudentDetector.forward()`:

```text
score = sigmoid(objectness) * sigmoid(centerness)
                         │
              top 300 across P3-P5
                         │
                decode l,t,r,b
                         │
            clip to image boundaries
                         │
        class-agnostic greedy NMS, IoU=0.6
                         │
                 at most 100 boxes
```

There is currently no pre-NMS score threshold; top-K bounds the work before NMS.
The decoder returns one variable-length `Detection` object per batch element.
NMS is implemented in pure PyTorch to avoid coupling this small post-processing
step to a platform-specific torchvision operator. Its data-dependent loop is
acceptable here precisely because the decoder is outside the exported and
quantized graph.

The score is a direct product, not the square root sometimes used by other FCOS
implementations. This matches the Phase-2 specification and should remain fixed
when comparing experiments.

Implementation: [`student_detector/decoder.py`](../student_detector/decoder.py).

## 9. Future semantic stage

The current model intentionally exposes no semantic extension API. The future
open-vocabulary stage will define its own per-region embedding contract after
the localization geometry and crop policy are selected.

## 10. Export and quantization boundary

[`torch.export`](https://docs.pytorch.org/docs/stable/user_guide/torch_compiler/export/programming_model.html)
captures tensor computation from example inputs into an ahead-of-time graph.
The raw model is designed accordingly:

- its output structure and per-level tensor shapes are fixed for a selected input
  resolution;
- P3/P4/P5 calls are written explicitly in the head;
- no target assignment, threshold-dependent proposal list, NMS loop, or semantic
  optionality occurs in `forward()`;
- resizing, addition, convolutions, normalization, ReLU, and stride scaling are
  ordinary tensor operations.

Both the stub model and real random-weight MobileNet detector pass
`torch.export.export()` at 384. The verified exported graph contains 694 nodes in
the current PyTorch/timm environment.

Quantization is intentionally not applied yet. The architecture prepares for it
by using conventional convolutional blocks, BatchNorm/ReLU patterns, simple
addition and nearest resize, and by keeping box decoding and NMS out of the graph.
Actual INT8 viability still requires a later PT2E/ExecuTorch conversion,
calibration data, numerical comparison, backend delegation inspection, and
on-device profiling.

### 10.1 `.pt2` and Model Explorer output workaround

PyTorch 2.6 can export `DetectorOutput`, but `torch.export.save()` cannot serialize
the custom named-tuple tree specification unless that type is registered through
an internal pytree mechanism. We avoid coupling production code to that private
registration API.

For saved `.pt2` inspection artifacts only, `scripts/inspect_phase2.py` wraps the
model in `ExportView`. It flattens the named output into the same nine tensors:
three objectness, three box-distance, and three centerness tensors. This changes
only the Python output container—not the operations, parameters, or tensor
values—and makes the program serializable and loadable by
[Model Explorer](https://github.com/google-ai-edge/model-explorer/wiki/4.-API-Guide).

Once the model is lowered to ExecuTorch, its official
[Model Explorer integration](https://docs.pytorch.org/executorch/stable/visualize.html)
can additionally visualize quantize/dequantize clusters and backend partitions.

## 11. Inspection and profiling

The optional `inspection` uv group contains `torchinfo`, `torchview`, Graphviz's
Python binding, Matplotlib, TensorBoard, and Model Explorer. Install it with:

```bash
uv sync --group dev --group inspection
```

Common commands are:

```bash
# Hierarchical layer, parameter, and multiply-add summary
uv run --group inspection python scripts/inspect_phase2.py summary

# Static eager execution graph
uv run --group inspection python scripts/inspect_phase2.py graph

# Interactive torch.export operator graph
uv run --group inspection python scripts/inspect_phase2.py model-explorer --launch

# C3-C5 and P3-P5 activation heatmaps
uv run --group inspection python scripts/inspect_phase2.py activations \
  --image path/to/image.jpg --checkpoint path/to/checkpoint.pt

# Parameter and activation histograms
uv run --group inspection python scripts/inspect_phase2.py tensorboard
```

See the dedicated [model inspection guide](model_inspection.md) for interpretation
and caveats. Random-weight activation maps validate plumbing but have no learned
semantic meaning. With an ImageNet-pretrained backbone but an untrained FPN,
C3-C5 are pretrained while P3-P5 are still random.

## 12. Verification status

Run the full Phase-2 checks from `projects/visual-inference`:

```bash
uv run --group dev pytest -q tests/student_detector
uv run python scripts/verify_phase2.py
```

Current status:

| Requirement | Status | Evidence |
| --- | --- | --- |
| C3/C4/C5 shapes | pass | real MobileNet test checks 80/160/256 and 48/24/12 at 384 |
| P3/P4/P5 output shapes | pass | tests cover 320 and 384 |
| fixed raw outputs | pass | nine dense P3-P5 tensors in a fixed named tuple |
| non-negative predicted distances | pass | tested after ReLU/stride scaling |
| shared prediction head | pass | one object tower and one regression tower, tested |
| ATSS positive per representable GT | pass | normal assignment and fallback tested |
| unrepresentable tiny GT reporting | pass | sub-grid box test checks reported index |
| overlapping-GT conflict resolution | pass | assignment test covers competing boxes |
| perfect target round-trip | pass | encode/decode equality test |
| clipping, NMS, maximum count | pass | decoder tests |
| real-model `torch.export` | pass | 384 export and execution test |
| saved Model Explorer `.pt2` | pass | flattened export artifact saves and reloads, 694 nodes |
| profiling entry point | pass locally | target Pi measurement still required |

The original Phase-2 suite contained 16 passing tests. Phase 3 extends the
detector suite. The CUDA/NVML warning seen on a CPU-only test runner is
environmental and does not indicate a model failure.

## 13. Known limitations and next decisions

Phase 3 has now implemented the preprocessing, losses, mask policy, optimizer,
BN policy, proposal metrics, and real-data audit described as open decisions in
the original Phase-2 handoff. See [`phase3.md`](phase3.md) for the rationale,
commands, and measured results.

Remaining architecture-level decisions are:

1. **Tiny boxes:** the real audit reports 1.63% grid-unrepresentable GTs after
   the tiny-object policy.
2. **ATSS fallback:** the real audit reports 9.20% fallback, concentrated in
   nuImages; inspect Stage-0 examples before changing agreed priors.
3. **Proposal quality:** the production training run has not occurred because
   this workstation has no usable CUDA device.
4. **Quantization:** fuse where supported, calibrate on representative images,
   compare FP32 versus INT8 proposal recall and box error, and inspect delegation.
5. **Target environment:** create a separate ARM/CPU dependency and deployment
   path after selecting the exact Raspberry Pi model and OS.
6. **Open-vocabulary stage:** define the per-region embedding and SigLIP
   distillation contract only after the localization baseline is stable.

## 14. Code map

| File | Responsibility |
| --- | --- |
| [`backbone.py`](../student_detector/backbone.py) | `timm` feature discovery and guarded C5 pruning |
| [`neck.py`](../student_detector/neck.py) | DSConv and 96-channel Lite FPN |
| [`head.py`](../student_detector/head.py) | shared decoupled head and fixed output contract |
| [`model.py`](../student_detector/model.py) | end-to-end raw detector |
| [`geometry.py`](../student_detector/geometry.py) | grid points, IoU, encode/decode |
| [`assigner.py`](../student_detector/assigner.py) | ATSS and representability reporting |
| [`decoder.py`](../student_detector/decoder.py) | scoring, top-K, clipping and class-agnostic NMS |
| [`verify_phase2.py`](../scripts/verify_phase2.py) | completion-criteria smoke checks |
| [`inspect_phase2.py`](../scripts/inspect_phase2.py) | summaries, graphs, activations and TensorBoard |

## 15. References

### Papers

1. Qin et al., [*MobileNetV4: Universal Models for the Mobile Ecosystem*](https://arxiv.org/abs/2404.10518), ECCV 2024.
2. Howard et al., [*MobileNets: Efficient Convolutional Neural Networks for Mobile Vision Applications*](https://arxiv.org/abs/1704.04861), 2017.
3. Lin et al., [*Feature Pyramid Networks for Object Detection*](https://openaccess.thecvf.com/content_cvpr_2017/html/Lin_Feature_Pyramid_Networks_CVPR_2017_paper.html), CVPR 2017.
4. Tian et al., [*FCOS: Fully Convolutional One-Stage Object Detection*](https://openaccess.thecvf.com/content_ICCV_2019/html/Tian_FCOS_Fully_Convolutional_One-Stage_Object_Detection_ICCV_2019_paper.html), ICCV 2019; [reference implementation](https://github.com/tianzhi0549/FCOS).
5. Zhang et al., [*Bridging the Gap Between Anchor-Based and Anchor-Free Detection via Adaptive Training Sample Selection*](https://openaccess.thecvf.com/content_CVPR_2020/html/Zhang_Bridging_the_Gap_Between_Anchor-Based_and_Anchor-Free_Detection_via_Adaptive_CVPR_2020_paper.html), CVPR 2020; [reference implementation](https://github.com/sfzhang15/ATSS).
6. Radford et al., [*Learning Transferable Visual Models From Natural Language Supervision*](https://proceedings.mlr.press/v139/radford21a.html), ICML 2021.

### Software and deployment documentation

1. [`timm` repository](https://github.com/huggingface/pytorch-image-models) and
   [feature-extraction documentation](https://huggingface.co/docs/timm/v1.0.15/en/feature_extraction).
2. PyTorch [`torch.export` overview](https://docs.pytorch.org/docs/main/user_guide/torch_compiler/export.html),
   [programming model](https://docs.pytorch.org/docs/stable/user_guide/torch_compiler/export/programming_model.html),
   and [PT2 archive specification](https://docs.pytorch.org/docs/main/user_guide/torch_compiler/export/pt2_archive.html).
3. PyTorch [module-fusion API](https://docs.pytorch.org/docs/main/generated/torch.ao.quantization.fuse_modules.fuse_modules.html).
4. Google AI Edge [Model Explorer](https://github.com/google-ai-edge/model-explorer)
   and its [PyTorch API guide](https://github.com/google-ai-edge/model-explorer/wiki/4.-API-Guide).
5. ExecuTorch [concepts](https://docs.pytorch.org/executorch/stable/concepts),
   [model visualization](https://docs.pytorch.org/executorch/stable/visualize.html),
   and [Inspector API](https://docs.pytorch.org/executorch/stable/model-inspector.html).
