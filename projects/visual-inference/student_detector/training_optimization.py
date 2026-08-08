"""Optimization primitives shared by proposal-training tasks."""

from __future__ import annotations

import copy
import math
import random
from collections.abc import Mapping
from typing import Any

import torch
from torch import Tensor, nn

from .config import Phase3Config


def set_reproducibility_seed(seed: int) -> None:
    random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def freeze_backbone_batch_norm(backbone: nn.Module) -> None:
    """Freeze running statistics while leaving affine parameters trainable."""
    for module in backbone.modules():
        if isinstance(module, nn.modules.batchnorm._BatchNorm):
            module.eval()


def set_backbone_trainable(model: nn.Module, trainable: bool) -> None:
    for parameter in model.backbone.parameters():  # type: ignore[attr-defined]
        parameter.requires_grad_(trainable)
    freeze_backbone_batch_norm(model.backbone)  # type: ignore[attr-defined]


class ExponentialMovingAverage:
    def __init__(self, model: nn.Module, decay: float, ramp_steps: int = 0) -> None:
        self.decay = decay
        self.ramp_steps = ramp_steps
        self.updates = 0
        self.initialization_weight = 1.0
        self.module = copy.deepcopy(model).eval()
        self.module.requires_grad_(False)

    def current_decay(self) -> float:
        if self.ramp_steps == 0:
            return self.decay
        return self.decay * (1 - math.exp(-self.updates / self.ramp_steps))

    @torch.no_grad()
    def update(self, model: nn.Module) -> None:
        self.updates += 1
        decay = self.current_decay()
        self.initialization_weight *= decay
        source = model.state_dict()
        for name, value in self.module.state_dict().items():
            current = source[name].detach()
            if value.is_floating_point():
                value.lerp_(current, 1 - decay)
            else:
                value.copy_(current)

    def restore_tracking(self, updates: int, initialization_weight: float) -> None:
        self.updates = updates
        self.initialization_weight = initialization_weight


class WarmupCosine:
    def __init__(
        self,
        optimizer: torch.optim.Optimizer,
        *,
        total_steps: int,
        warmup_steps: int,
        min_ratio: float,
    ) -> None:
        self.optimizer = optimizer
        self.total_steps = max(total_steps, 1)
        self.warmup_steps = min(warmup_steps, self.total_steps)
        self.min_ratio = min_ratio
        self.base_lrs = [group["lr"] for group in optimizer.param_groups]
        self.step_number = 0
        self._apply()

    def _ratio(self) -> float:
        if self.warmup_steps and self.step_number < self.warmup_steps:
            return max((self.step_number + 1) / self.warmup_steps, 1e-3)
        progress = (self.step_number - self.warmup_steps) / max(
            self.total_steps - self.warmup_steps, 1
        )
        progress = min(max(progress, 0.0), 1.0)
        return self.min_ratio + (1 - self.min_ratio) * (
            1 + math.cos(math.pi * progress)
        ) * 0.5

    def _apply(self) -> None:
        ratio = self._ratio()
        for group, base_lr in zip(
            self.optimizer.param_groups, self.base_lrs, strict=True
        ):
            group["lr"] = base_lr * ratio

    def step(self) -> None:
        self.step_number += 1
        self._apply()

    def state_dict(self) -> dict[str, Any]:
        return {"step_number": self.step_number}

    def load_state_dict(self, state: Mapping[str, Any]) -> None:
        self.step_number = int(state["step_number"])
        self._apply()


def build_detector_optimizer(
    model: nn.Module,
    config: Phase3Config,
    *,
    world_size: int = 1,
) -> torch.optim.AdamW:
    effective_batch = (
        config.data.batch_size
        * config.schedule.accumulation_steps
        * world_size
    )
    scaled_lr = config.schedule.head_learning_rate * math.sqrt(
        effective_batch / config.schedule.reference_effective_batch
    )
    return torch.optim.AdamW(
        [
            {
                "params": list(model.backbone.parameters()),  # type: ignore[attr-defined]
                "lr": scaled_lr * config.schedule.backbone_lr_multiplier,
                "name": "backbone",
            },
            {
                "params": [
                    *model.fpn.parameters(),  # type: ignore[attr-defined]
                    *model.head.parameters(),  # type: ignore[attr-defined]
                ],
                "lr": scaled_lr,
                "name": "fpn_head",
            },
        ],
        weight_decay=config.schedule.weight_decay,
    )


def optimizer_to(optimizer: torch.optim.Optimizer, device: torch.device) -> None:
    """Move optimizer tensor state after loading a CPU checkpoint."""
    for state in optimizer.state.values():
        for key, value in state.items():
            if isinstance(value, Tensor):
                state[key] = value.to(device)
