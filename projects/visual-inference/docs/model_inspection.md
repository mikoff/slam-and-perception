# Inspecting and visualizing the student detector

There is no single visualization that explains a neural network. Use a different
view depending on the question:

| Question | Tool | Result |
| --- | --- | --- |
| Which modules exist and how expensive are they? | `torchinfo` | terminal table of shapes, parameters, and estimated multiply-adds |
| Which modules call which other modules? | `torchview` + Graphviz | static SVG execution graph |
| What operators will the exported program execute? | Model Explorer | interactive, hierarchical `torch.export` graph |
| Where are intermediate features spatially active? | PyTorch feature extraction + Matplotlib | C3-C5 and P3-P5 heatmaps |
| How do weights, activations, and losses change during training? | TensorBoard | interactive time-series, image, and histogram dashboard |
| What is inside the exported program? | `torch.export` graph, later Model Explorer | operator-level deployment graph |

The first four workflows are supported by `scripts/inspect_phase2.py`.

## Install the inspection environment

The visualization packages are in a separate uv group so that they are not part
of the target-device runtime:

```bash
uv sync --group dev --group inspection
```

This installs:

- [`torchinfo`](https://github.com/TylerYep/torchinfo) for summary tables;
- [`torchview`](https://github.com/mert-kurttutan/torchview) and its Python
  Graphviz binding for model graphs;
- [Model Explorer](https://github.com/google-ai-edge/model-explorer) for an
  interactive `ExportedProgram` graph;
- [TensorBoard through PyTorch](https://docs.pytorch.org/docs/stable/tensorboard.html)
  for interactive logs;
- Matplotlib for activation images.

`torchview` also needs the system `dot` executable. On Debian/Ubuntu it is:

```bash
sudo apt install graphviz
dot -V
```

The uv package named `graphviz` is only the Python interface; it cannot render an
SVG without the system executable. Do not install these tools in the minimal Pi
inference image unless you intend to inspect the model directly on the Pi.

All commands below run from `projects/visual-inference` and use random detector
weights unless `--checkpoint` or `--pretrained-backbone` is supplied.

## 1. Read the module and cost summary

```bash
uv run --group inspection python scripts/inspect_phase2.py summary \
  --image-size 384 --depth 4
```

Start with these columns:

- **Input/Output Shape** checks channel counts and spatial reductions.
- **Param #** shows where model storage is spent.
- **Mult-Adds** estimates arithmetic work. It is useful for comparing two
  architectures, but it is not a latency prediction. Memory traffic, kernel
  implementation, thread count, and quantization can dominate on a Pi.
- **Recursive** on the shared head is expected: the same module and parameters
  are called once for P3, P4, and P5.

Use `--depth 2` for the backbone/FPN/head overview. Increase the depth when you
want to inspect MobileNet blocks or the depthwise/pointwise structure. A very
deep table becomes harder to learn from than a focused one.

You can also inspect one submodule in a Python session:

```python
import torch
from torchinfo import summary
from student_detector import StudentDetector

model = StudentDetector(pretrained_backbone=False).eval()
summary(model.fpn, input_data=[(
    # C3, C4 and C5 examples
    torch.randn(1, 80, 48, 48),
    torch.randn(1, 160, 24, 24),
    torch.randn(1, 256, 12, 12),
)])
```

## 2. Render the execution graph

```bash
uv run --group inspection python scripts/inspect_phase2.py graph \
  --image-size 384 --depth 4
```

Open `artifacts/model_inspection/student_detector.svg`. The graph is produced by
executing the model with a sample input, so it shows the actual branch and module
calls used by that input. In particular, you can see:

1. C5 projected into P5;
2. P5 resized and added to the C4 lateral;
3. that result resized and added to the C3 lateral;
4. the same object and regression towers reused at all three levels.

If the image is too large, lower `--depth`. If a detail is hidden, raise it. The
operator-level `torch.export` graph has hundreds of nodes and is usually a poor
first architecture diagram; use it later to diagnose export or quantization.

## 3. Explore the exported graph interactively

This is the appropriate workflow when the question is about the graph captured
for deployment rather than the eager Python module hierarchy:

```bash
uv run --group inspection python scripts/inspect_phase2.py model-explorer \
  --image-size 384 --launch
```

The command performs the equivalent of:

```python
import model_explorer
import torch

# The project script wraps DetectorOutput as a plain nine-tensor tuple because
# PyTorch 2.6 cannot save an unregistered custom NamedTuple TreeSpec to .pt2.
export_view = ExportView(model)
exported_program = torch.export.export(export_view, (dummy_input,))
model_explorer.visualize_pytorch(
    "StudentDetector", exported_program=exported_program
)
```

`ExportView` is defined in `scripts/inspect_phase2.py`. It changes only the
output container, not the operations or tensor values shown in the graph.

The viewer opens at `http://localhost:8080` and the command remains active until
you stop it with `Ctrl+C`. It lets you expand/collapse the original module
hierarchy, flatten the graph, search operators, and inspect tensor shapes and
connections. This is substantially easier to navigate than printing all 694
nodes of this detector's exported graph.

Without `--launch`, the command writes
`artifacts/model_inspection/student_detector_384.pt2`. Open it later with:

```bash
uv run --group inspection model-explorer \
  artifacts/model_inspection/student_detector_384.pt2
```

The suggested import `executorch.sdk.model_explorer` belongs to an older API
layout. Current ExecuTorch documentation uses visualization helpers under
`executorch.devtools.visualization.visualization_utils`. Those helpers are most
valuable after conversion to Edge/ExecuTorch dialect, when they can highlight
quantize/dequantize clusters and backend partitions. We will add that view in
the deployment phase; installing the full ExecuTorch compiler is unnecessary
for visualizing the Phase-2 `torch.export` graph.

## 4. Visualize intermediate feature maps

With an image:

```bash
uv run --group inspection python scripts/inspect_phase2.py activations \
  --image path/to/image.jpg --image-size 384 \
  --checkpoint path/to/student_detector.pt
```

Without a checkpoint, you may inspect only the pretrained backbone:

```bash
uv run --group inspection python scripts/inspect_phase2.py activations \
  --image path/to/image.jpg --pretrained-backbone
```

The output is `artifacts/model_inspection/activations.png`. Each displayed map is
the mean absolute response across channels, normalized independently to `[0, 1]`
for visibility. This is a diagnostic image, not a calibrated confidence map.

Interpret the stages cautiously:

- C3 has the finest grid and usually retains edges, boundaries, and local
  texture.
- C4 and C5 trade spatial detail for larger receptive fields and more contextual
  features.
- P5 starts from C5; P4 and P3 combine deep context with finer lateral features.
- A bright cell says that the aggregate feature magnitude is large. It does not
  prove that the cell represents a particular class or object.

Random-weight features have no learned semantic meaning. With
`--pretrained-backbone`, C3-C5 use ImageNet knowledge, but P3-P5 remain random
until the detector is trained. Meaningful comparison also requires the same
resize, color conversion, and normalization used during training.

For one particular channel rather than a channel average, use a forward hook:

```python
captured = {}

def save_p3(_module, _inputs, output):
    captured["P3"] = output.detach().cpu()

handle = model.fpn.output3.register_forward_hook(save_p3)
with torch.inference_mode():
    model(image_batch)
handle.remove()  # important: otherwise repeated calls keep invoking the hook

p3_channel_17 = captured["P3"][0, 17]
```

Hooks are excellent for experiments but should not be left active during normal
training or export because retaining activation tensors increases memory use.

## 5. Inspect detector outputs

The output maps are easier to interpret after applying the same scoring rule as
inference:

```python
with torch.inference_mode():
    output = model(image_batch)

p3_objectness = output.objectness[0].sigmoid()
p3_centerness = output.centerness[0].sigmoid()
p3_score = p3_objectness * p3_centerness
p3_distances_px = output.box_distances[0]
```

Plot `p3_score[0, 0]` as a heatmap. A high value means “this grid location is a
good centered object proposal,” not “this is class X.” The four distance channels
are left, top, right, and bottom distances in input pixels. Use the decoder to
turn them into boxes; viewing each raw distance channel alone is rarely intuitive.

## 6. Use TensorBoard for distributions and training history

Create a one-step inspection log:

```bash
uv run --group inspection python scripts/inspect_phase2.py tensorboard \
  --image path/to/image.jpg --checkpoint path/to/student_detector.pt

uv run --group inspection tensorboard \
  --logdir artifacts/model_inspection/tensorboard
```

Open the address printed by TensorBoard, normally `http://localhost:6006`. The
script records parameter histograms, C3-C5/P3-P5 activation histograms, the input,
and aggregate activation maps. PyTorch's `SummaryWriter` supports scalars,
histograms, images, graphs, and embeddings, so the same API can be added to the
training loop.

Useful training signals include:

- total, objectness, box, and centerness losses;
- positive ATSS locations per image and per level;
- regression-scale values for P3/P4/P5;
- gradient norms and learning rate;
- weight and activation histograms every few hundred steps;
- validation proposal recall before and after NMS.

Do not histogram every parameter and activation on every batch. Logging is CPU,
disk, and synchronization work; sample it periodically.

## 7. Inspect the exported graph as text

For a textual operator graph, no extra package is needed:

```python
import torch
from student_detector import StudentDetector

model = StudentDetector(pretrained_backbone=False).eval()
example = torch.randn(1, 3, 384, 384)
exported = torch.export.export(model, (example,))
print(exported.graph)
```

For later ExecuTorch deployment, PyTorch also documents
[`visualize_with_clusters`](https://docs.pytorch.org/executorch/stable/visualize.html),
which highlights quantization clusters and backend partitions in lowered graphs.

## What not to over-interpret

- A pretty activation map is not evidence that the detector works. Measure
  proposal recall and localization quality.
- Feature-map colors are independently normalized, so brightness cannot be
  compared quantitatively between panels.
- `train()` and `eval()` differ because of BatchNorm. Use `eval()` for stable
  inspection of a checkpoint.
- Grad-CAM and similar attribution tools need a meaningful scalar target. The
  current detector is class-agnostic and initially untrained, so Grad-CAM is less
  useful than proposal-score maps. Captum can be added later when semantic scores
  or a specific trained proposal target exist.
- Graph and MAC tools describe the PyTorch model. The final INT8 runtime may fuse
  or rewrite operators, so inspect and profile the deployed artifact as well.
