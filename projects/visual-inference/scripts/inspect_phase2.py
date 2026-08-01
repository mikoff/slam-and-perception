"""Inspect the Phase-2 model structure, graph, activations, and distributions."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import torch
from PIL import Image

from student_detector import StudentDetector


DEFAULT_OUTPUT = Path("artifacts/model_inspection")


class ExportView(torch.nn.Module):
    """Give torch.export a standard tuple output that torch 2.6 can serialize."""

    def __init__(self, model: StudentDetector) -> None:
        super().__init__()
        self.model = model

    def forward(self, images: torch.Tensor) -> tuple[torch.Tensor, ...]:
        output = self.model(images)
        return (
            *output.objectness,
            *output.box_distances,
            *output.centerness,
        )


def add_model_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--image-size", type=int, default=384, choices=(320, 384))
    parser.add_argument(
        "--checkpoint",
        type=Path,
        help="StudentDetector state_dict, or a checkpoint containing a 'model' state_dict.",
    )
    parser.add_argument(
        "--pretrained-backbone",
        action="store_true",
        help="Download/use timm ImageNet weights when no detector checkpoint is supplied.",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    summary = subparsers.add_parser("summary", help="Print layer shapes and parameter/MAC counts.")
    add_model_arguments(summary)
    summary.add_argument("--depth", type=int, default=4)

    graph = subparsers.add_parser("graph", help="Render the executed module graph as SVG.")
    add_model_arguments(graph)
    graph.add_argument("--depth", type=int, default=4)
    graph.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)

    model_explorer = subparsers.add_parser(
        "model-explorer", help="Export the operator graph and optionally open Model Explorer."
    )
    add_model_arguments(model_explorer)
    model_explorer.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    model_explorer.add_argument(
        "--launch",
        action="store_true",
        help="Start the blocking local Model Explorer server and open a browser.",
    )

    activations = subparsers.add_parser(
        "activations", help="Save C3-C5 and P3-P5 activation heatmaps."
    )
    add_model_arguments(activations)
    activations.add_argument("--image", type=Path, help="Optional RGB image; otherwise use noise.")
    activations.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)

    tensorboard = subparsers.add_parser(
        "tensorboard", help="Log parameter and activation histograms for TensorBoard."
    )
    add_model_arguments(tensorboard)
    tensorboard.add_argument("--image", type=Path, help="Optional RGB image; otherwise use noise.")
    tensorboard.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT / "tensorboard")
    return parser.parse_args()


def load_model(args: argparse.Namespace) -> StudentDetector:
    model = StudentDetector(
        pretrained_backbone=args.pretrained_backbone and args.checkpoint is None
    )
    if args.checkpoint is not None:
        checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
        state_dict = checkpoint.get("model", checkpoint) if isinstance(checkpoint, dict) else checkpoint
        model.load_state_dict(state_dict)
    return model.eval()


def load_input(image_path: Path | None, image_size: int) -> tuple[torch.Tensor, torch.Tensor]:
    """Return normalized model input and displayable RGB input."""
    if image_path is None:
        torch.manual_seed(0)
        display = torch.rand(3, image_size, image_size)
    else:
        resampling = getattr(Image, "Resampling", Image).BILINEAR
        image = Image.open(image_path).convert("RGB").resize(
            (image_size, image_size), resampling
        )
        display = torch.from_numpy(np.asarray(image).copy()).permute(2, 0, 1)
        display = display.to(torch.float32) / 255.0

    mean = torch.tensor((0.485, 0.456, 0.406)).view(3, 1, 1)
    std = torch.tensor((0.229, 0.224, 0.225)).view(3, 1, 1)
    return ((display - mean) / std).unsqueeze(0), display


def normalized_map(feature: torch.Tensor) -> torch.Tensor:
    """Collapse channels to one display map and normalize only for visualization."""
    heatmap = feature[0].abs().mean(dim=0).detach().cpu()
    minimum, maximum = heatmap.aminmax()
    return (heatmap - minimum) / (maximum - minimum).clamp(min=1e-12)


def collect_features(
    model: StudentDetector, model_input: torch.Tensor
) -> tuple[tuple[torch.Tensor, ...], tuple[torch.Tensor, ...]]:
    with torch.inference_mode():
        backbone_features = model.backbone(model_input)
        pyramid_features = model.fpn(backbone_features)
    return backbone_features, pyramid_features


def print_summary(args: argparse.Namespace) -> None:
    from torchinfo import summary

    model = load_model(args)
    summary(
        model,
        input_size=(1, 3, args.image_size, args.image_size),
        depth=args.depth,
        device="cpu",
        col_names=("input_size", "output_size", "num_params", "mult_adds"),
        verbose=1,
    )


def render_graph(args: argparse.Namespace) -> None:
    from torchview import draw_graph

    model = load_model(args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    draw_graph(
        model,
        input_size=(1, 3, args.image_size, args.image_size),
        depth=args.depth,
        device="cpu",
        expand_nested=True,
        save_graph=True,
        filename="student_detector",
        directory=str(args.output_dir),
        graph_name="StudentDetector",
    )
    print(f"graph: {args.output_dir / 'student_detector.svg'}")


def explore_exported_graph(args: argparse.Namespace) -> None:
    model = load_model(args)
    example = torch.randn(1, 3, args.image_size, args.image_size)
    exported_program = torch.export.export(ExportView(model), (example,))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output_dir / f"student_detector_{args.image_size}.pt2"
    torch.export.save(exported_program, output_path)
    print(f"exported graph: {output_path}")

    if args.launch:
        import model_explorer

        print("starting Model Explorer at http://localhost:8080 (Ctrl+C to stop)")
        model_explorer.visualize_pytorch(
            "StudentDetector", exported_program=exported_program
        )
    else:
        print(
            "add --launch for the interactive viewer, or run: "
            f"uv run --group inspection model-explorer {output_path}"
        )


def save_activations(args: argparse.Namespace) -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/visual-inference-matplotlib")
    import matplotlib.pyplot as plt

    model = load_model(args)
    model_input, display = load_input(args.image, args.image_size)
    backbone_features, pyramid_features = collect_features(model, model_input)
    features = (*backbone_features, *pyramid_features)
    names = ("C3", "C4", "C5", "P3", "P4", "P5")

    figure, axes = plt.subplots(2, 4, figsize=(14, 7), constrained_layout=True)
    axes[0, 0].imshow(display.permute(1, 2, 0))
    axes[0, 0].set_title("input")
    axes[1, 0].axis("off")
    for axis, name, feature in zip(axes[:, 1:].reshape(-1), names, features, strict=True):
        axis.imshow(normalized_map(feature), cmap="magma", vmin=0, vmax=1)
        axis.set_title(f"{name}: {tuple(feature.shape[1:])}")
        axis.axis("off")
    axes[0, 0].axis("off")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output_dir / "activations.png"
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    print(f"activations: {output_path}")
    if args.checkpoint is None and not args.pretrained_backbone:
        print("note: these are random-weight activations; use a checkpoint for semantic interpretation")


def log_tensorboard(args: argparse.Namespace) -> None:
    from torch.utils.tensorboard import SummaryWriter

    model = load_model(args)
    model_input, display = load_input(args.image, args.image_size)
    backbone_features, pyramid_features = collect_features(model, model_input)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    writer = SummaryWriter(log_dir=str(args.output_dir))
    writer.add_image("input/rgb", display, global_step=0)
    for name, parameter in model.named_parameters():
        writer.add_histogram(f"parameters/{name}", parameter.detach().cpu(), global_step=0)
    for name, feature in zip(
        ("C3", "C4", "C5", "P3", "P4", "P5"),
        (*backbone_features, *pyramid_features),
        strict=True,
    ):
        writer.add_histogram(f"activations/{name}", feature.detach().cpu(), global_step=0)
        writer.add_image(f"activation_maps/{name}", normalized_map(feature), 0, dataformats="HW")
    writer.close()
    print(f"TensorBoard log: {args.output_dir}")
    print(f"launch: uv run --group inspection tensorboard --logdir {args.output_dir}")


def main() -> None:
    args = parse_args()
    if args.command == "summary":
        print_summary(args)
    elif args.command == "graph":
        render_graph(args)
    elif args.command == "model-explorer":
        explore_exported_graph(args)
    elif args.command == "activations":
        save_activations(args)
    elif args.command == "tensorboard":
        log_tensorboard(args)


if __name__ == "__main__":
    main()
