"""MobileNetV4 feature extraction selected by spatial reduction."""

from __future__ import annotations

from collections.abc import Sequence

from torch import Tensor, nn


PYRAMID_REDUCTIONS = (8, 16, 32)
EXPECTED_CHANNELS = (80, 160, 256)
P2_PYRAMID_REDUCTIONS = (4, 8, 16, 32)
P2_EXPECTED_CHANNELS = (48, 80, 160, 256)
DEFAULT_MODEL_NAME = "mobilenetv4_conv_medium.e500_r256_in1k"


def _last_conv_out_channels(module: nn.Module) -> int | None:
    for child in reversed(tuple(module.modules())):
        if isinstance(child, nn.Conv2d):
            return child.out_channels
    return None


def locate_pyramid_levels(
    reductions: Sequence[int],
    channels: Sequence[int],
    desired_reductions: Sequence[int] = PYRAMID_REDUCTIONS,
) -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Find output positions and channels for the requested feature strides."""
    if len(reductions) != len(channels):
        raise ValueError("feature reductions and channels must have equal length")

    positions: list[int] = []
    selected_channels: list[int] = []
    for desired in desired_reductions:
        matches = [
            index for index, reduction in enumerate(reductions) if reduction == desired
        ]
        if not matches:
            raise ValueError(
                f"backbone has no stride-{desired} feature; available reductions: "
                f"{list(reductions)}"
            )
        # A backbone may expose more than one block at a reduction. The final block
        # is the stage output that should feed the pyramid.
        position = matches[-1]
        positions.append(position)
        selected_channels.append(int(channels[position]))

    return tuple(positions), tuple(selected_channels)


class MobileNetV4Backbone(nn.Module):
    """Classification-free timm backbone returning C3, C4 and C5."""

    out_channels: tuple[int, ...]
    reductions: tuple[int, ...]

    def __init__(
        self,
        model_name: str = DEFAULT_MODEL_NAME,
        *,
        pretrained: bool = True,
        verify_expected_channels: bool = True,
        desired_reductions: tuple[int, ...] = PYRAMID_REDUCTIONS,
    ) -> None:
        super().__init__()
        try:
            import timm
        except (
            ImportError
        ) as exc:  # pragma: no cover - exercised only in a broken environment
            raise ImportError(
                "MobileNetV4Backbone requires the 'timm' package"
            ) from exc

        self.body = timm.create_model(
            model_name,
            pretrained=pretrained,
            features_only=True,
        )
        reductions = tuple(int(value) for value in self.body.feature_info.reduction())
        channels = tuple(int(value) for value in self.body.feature_info.channels())
        positions, selected_channels = locate_pyramid_levels(
            reductions, channels, desired_reductions
        )
        expected_channels = tuple(
            {4: 48, 8: 80, 16: 160, 32: 256}[stride] for stride in desired_reductions
        )

        # In current timm, MobileNetV4's feature wrapper groups the final
        # 256->960 classification expansion into its last stride-32 stage. The
        # detector wants the preceding 256-channel convolutional stage. Discover
        # the expansion from feature_info, verify its predecessor, and prune it so
        # it also costs no MACs at inference time.
        if (
            selected_channels[:-1] == expected_channels[:-1]
            and selected_channels[-1] != 256
        ):
            feature_dicts = self.body.feature_info.get_dicts()
            selected_info = [feature_dicts[position] for position in positions]
            c5_info = selected_info[-1]
            module_parts = str(c5_info["module"]).split(".")
            can_prune_expansion = (
                len(module_parts) == 2
                and module_parts[0] == "blocks"
                and module_parts[1].isdigit()
                and hasattr(self.body, "blocks")
                and hasattr(self.body, "_stage_out_idx")
            )
            if can_prune_expansion:
                expansion_index = int(module_parts[1])
                previous_index = expansion_index - 1
                previous_channels = (
                    _last_conv_out_channels(self.body.blocks[previous_index])
                    if previous_index >= 0
                    else None
                )
                if previous_channels == 256:
                    self.body.blocks = self.body.blocks[:expansion_index]
                    # MobileNetV3Features checks i + 1 against this mapping.
                    self.body._stage_out_idx = {
                        **{
                            int(info["stage"]): output_index
                            for output_index, info in enumerate(selected_info[:-1])
                        },
                        expansion_index: len(selected_info) - 1,
                    }
                    positions = tuple(range(len(selected_info)))
                    selected_channels = expected_channels

        if verify_expected_channels and selected_channels != expected_channels:
            raise ValueError(
                f"{model_name} produced channels {selected_channels} at reductions "
                f"{desired_reductions}, expected {expected_channels}. Check the "
                "installed timm version/model tag."
            )

        self._positions = positions
        self.out_channels = selected_channels
        self.reductions = desired_reductions

    def forward(self, images: Tensor) -> tuple[Tensor, ...]:
        features = self.body(images)
        return tuple(features[position] for position in self._positions)


class StubBackbone(nn.Module):
    """Small deterministic backbone used by unit tests and interface experiments."""

    out_channels = EXPECTED_CHANNELS
    reductions = PYRAMID_REDUCTIONS

    def __init__(self) -> None:
        super().__init__()
        self.c3 = nn.Conv2d(3, 80, kernel_size=3, stride=8, padding=1)
        self.c4 = nn.Conv2d(80, 160, kernel_size=3, stride=2, padding=1)
        self.c5 = nn.Conv2d(160, 256, kernel_size=3, stride=2, padding=1)

    def forward(self, images: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        c3 = self.c3(images)
        c4 = self.c4(c3)
        c5 = self.c5(c4)
        return c3, c4, c5


class StubP2Backbone(nn.Module):
    """Four-level deterministic backbone used by P2-P5 tests."""

    out_channels = P2_EXPECTED_CHANNELS
    reductions = P2_PYRAMID_REDUCTIONS

    def __init__(self) -> None:
        super().__init__()
        self.c2 = nn.Conv2d(3, 48, kernel_size=3, stride=4, padding=1)
        self.c3 = nn.Conv2d(48, 80, kernel_size=3, stride=2, padding=1)
        self.c4 = nn.Conv2d(80, 160, kernel_size=3, stride=2, padding=1)
        self.c5 = nn.Conv2d(160, 256, kernel_size=3, stride=2, padding=1)

    def forward(self, images: Tensor) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        c2 = self.c2(images)
        c3 = self.c3(c2)
        c4 = self.c4(c3)
        c5 = self.c5(c4)
        return c2, c3, c4, c5
