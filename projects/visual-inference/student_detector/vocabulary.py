"""Variable text vocabulary bank for zero-runtime-text-encoder open-vocabulary detection."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import torch
from torch import Tensor
import torch.nn.functional as F

from .siglip import DEFAULT_PROMPT_TEMPLATES, SiglipTeacher

DEFAULT_CATEGORY_SYNONYMS: dict[str, tuple[str, ...]] = {
    "traffic_sign": (
        "traffic sign",
        "road sign",
        "street sign",
        "directional road sign",
        "traffic arrow sign",
        "stop sign",
        "speed limit sign",
    ),
    "signboard": (
        "storefront signboard",
        "commercial signboard",
        "shop sign",
        "billboard",
        "signboard",
    ),
    "traffic_light": (
        "traffic light",
        "traffic signal",
        "stoplight",
    ),
    "window": (
        "building window",
        "house window",
        "room window",
        "architectural window",
        "glass window",
    ),
    "door": (
        "building door",
        "entrance door",
        "house door",
        "front door",
        "doorway",
    ),
    "building": (
        "building",
        "house",
        "architectural building",
        "office building",
    ),
    "tree": (
        "tree",
        "street tree",
        "palm tree",
        "tree foliage",
    ),
    "fence": (
        "fence",
        "metal fence",
        "wooden fence",
        "guardrail fence",
    ),
    "chair": (
        "chair",
        "armchair",
        "office chair",
        "sofa chair",
    ),
    "table": (
        "table",
        "desk",
        "dining table",
        "coffee table",
    ),
    "cabinet": (
        "cabinet",
        "cupboard",
        "wardrobe",
        "bookshelf cabinet",
    ),
    "wheeled_transport": (
        "trailer",
        "cargo trailer",
        "wheeled transport vehicle",
    ),
    "traffic_cone": (
        "traffic cone",
        "construction cone",
        "safety cone",
        "orange pylon",
    ),
    "construction_barrier": (
        "construction barrier",
        "traffic barrier",
        "roadside construction fence",
        "jersey barrier",
        "road barricade",
        "temporary hoarding barrier",
    ),
    "license_plate": (
        "license plate",
        "vehicle registration plate",
        "number plate",
    ),
    "pole": (
        "utility pole",
        "metal pole",
        "street pole",
        "sign pole",
    ),
    "street_light": (
        "street light",
        "outdoor street lamp post",
        "roadway street light pole",
        "highway lamp post",
    ),
    "indoor_lamp_or_light_fixture": (
        "indoor lamp",
        "table lamp",
        "bedside lamp",
        "ceiling light fixture",
        "chandelier",
        "wall sconce",
        "skylight window",
    ),
    "bed_and_bedding": (
        "bed",
        "bedroom bed",
        "mattress and pillows",
        "bedspread",
        "blanket on a bed",
    ),
    "vegetation": (
        "roadside bush",
        "shrubs and bushes",
        "hedge vegetation",
        "grass and weeds",
        "ground foliage",
    ),
    "ego_platform_bodywork": (
        "ego vehicle hood",
        "car bonnet hood at bottom of camera",
        "dashboard hood",
    ),
    "building_wall": (
        "plain building wall",
        "wall molding",
        "interior ceiling",
        "room floor",
        "roof surface",
    ),
    "pillar": (
        "concrete pillar",
        "support pillar",
        "architectural column",
        "structural column",
    ),
    "pedestrian": (
        "pedestrian",
        "person",
        "human walking",
        "person on the street",
    ),
    "cyclist": (
        "cyclist",
        "bicycle",
        "motorcycle",
        "person riding a bike",
    ),
    "trash_bin": (
        "litter bin",
        "waste container",
        "garbage can",
        "rubbish bin",
        "trash bin",
    ),
    "dog": (
        "furry pet dog",
        "furry barking dog",
        "golden retriever dog",
        "terrier pet dog",
    ),
    "fire_hydrant": (
        "red metal fireplug",
        "fire hydrant water valve",
    ),
    "cart": (
        "shopping cart",
        "supermarket shopping trolley",
    ),
    "stairs": (
        "staircase steps",
        "flight of stairs",
        "concrete stairway steps",
    ),
    "debris": (
        "pile of trash debris on ground",
        "scattered rubble litter",
        "damaged debris pile",
    ),
    "flowerpot": (
        "flowerpot",
        "potted plant",
        "plant pot",
    ),
}


@dataclass
class VocabularyBank:
    """Serialized open-vocabulary text embedding cache for fast edge inference.

    /// @brief Pre-encoded semantic vocabulary bank enabling zero-text-encoder edge classification.
    /// @param categories Ordered tuple of category label strings.
    /// @param embeddings Normalized unit L2 text representations of shape [C, D].
    /// @param logit_scale SigLIP temperature scale parameter.
    /// @param logit_bias SigLIP logit bias offset.
    /// @param class_priors Optional prior class frequency distribution [C] for logit adjustment.
    /// @param centroid Optional empirical feature centroid [1, D] for centering / de-biasing.
    """

    categories: tuple[str, ...]
    embeddings: Tensor
    logit_scale: float
    logit_bias: float
    class_priors: Tensor | None = None
    centroid: Tensor | None = None

    def __post_init__(self) -> None:
        if self.embeddings.ndim != 2:
            raise ValueError(f"embeddings must be 2D [C, D], got shape {self.embeddings.shape}")
        if len(self.categories) != self.embeddings.shape[0]:
            raise ValueError(
                f"categories length ({len(self.categories)}) must match embedding count ({self.embeddings.shape[0]})"
            )

    @property
    def num_classes(self) -> int:
        """Return the number of categories in this vocabulary bank."""
        return len(self.categories)

    @property
    def embedding_dim(self) -> int:
        """Return the feature embedding dimension."""
        return self.embeddings.shape[1]

    def to(self, device: torch.device | str) -> VocabularyBank:
        """Move embedding tensors to target device."""
        return VocabularyBank(
            categories=self.categories,
            embeddings=self.embeddings.to(device),
            logit_scale=self.logit_scale,
            logit_bias=self.logit_bias,
            class_priors=self.class_priors.to(device) if self.class_priors is not None else None,
            centroid=self.centroid.to(device) if self.centroid is not None else None,
        )

    def classify(
        self,
        region_embeddings: Tensor,
        mode: str = "softmax",
        temperature: float = 1.0,
        use_centroid_subtraction: bool = False,
        logit_adjustment_tau: float = 0.0,
    ) -> tuple[Tensor, Tensor]:
        """Classify region embeddings against this vocabulary bank.

        /// @brief Dot-product matrix multiplication against pre-encoded text embeddings.
        /// @param region_embeddings Tensor of shape [N, D] with unit L2 norm.
        /// @param mode Probability mode: 'softmax' (categorical distribution) or 'sigmoid' (raw SigLIP).
        /// @param temperature Softmax temperature from `fit_temperature` (1.0 = uncalibrated).
        /// @param use_centroid_subtraction If True and self.centroid is set, centers embeddings before similarity.
        /// @param logit_adjustment_tau Long-tail logit adjustment weight (Menon et al., 2021).
        /// @return Tuple of (logits [N, C], probabilities [N, C]).
        """
        if region_embeddings.numel() == 0 or self.embeddings.numel() == 0:
            c = len(self.categories)
            empty = torch.empty((region_embeddings.shape[0], c), dtype=torch.float32, device=region_embeddings.device)
            return empty, empty

        # Ensure same device and float32
        text_w = self.embeddings.to(region_embeddings.device, dtype=torch.float32)
        reg_x = region_embeddings.to(torch.float32)

        # Centroid centering for anisotropic manifold correction
        if use_centroid_subtraction and self.centroid is not None:
            cent = self.centroid.to(reg_x.device, dtype=torch.float32)
            reg_x = F.normalize(reg_x - cent, p=2.0, dim=-1)

        # Dot product [N, D] @ [D, C] -> [N, C]
        cosine_sim = torch.matmul(reg_x, text_w.t())
        scale = torch.exp(torch.tensor(self.logit_scale, dtype=torch.float32, device=region_embeddings.device))
        bias = torch.tensor(self.logit_bias, dtype=torch.float32, device=region_embeddings.device)

        logits = cosine_sim * scale + bias

        # Logit adjustment for class frequency imbalance
        if logit_adjustment_tau > 0.0 and self.class_priors is not None:
            priors = self.class_priors.to(region_embeddings.device, dtype=torch.float32)
            logits = logits - logit_adjustment_tau * torch.log(priors.clamp(min=1e-8))

        if mode == "softmax":
            probabilities = torch.softmax(logits / temperature, dim=-1)
        else:
            probabilities = torch.sigmoid(logits)
        return logits, probabilities

    def save(self, path: Path | str) -> None:
        """Serialize this vocabulary bank to a compact PyTorch checkpoint."""
        target_path = Path(path)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "categories": list(self.categories),
            "embeddings": self.embeddings.detach().cpu().to(torch.float32),
            "logit_scale": float(self.logit_scale),
            "logit_bias": float(self.logit_bias),
            "class_priors": self.class_priors.detach().cpu().to(torch.float32) if self.class_priors is not None else None,
            "centroid": self.centroid.detach().cpu().to(torch.float32) if self.centroid is not None else None,
            "schema_version": "1.1",
        }
        torch.save(payload, target_path)

    @classmethod
    def load(cls, path: Path | str, device: torch.device | str = "cpu") -> VocabularyBank:
        """Load a serialized vocabulary bank from disk."""
        target_path = Path(path)
        if not target_path.exists():
            raise FileNotFoundError(f"Vocabulary bank file not found: {target_path}")
        payload = torch.load(target_path, map_location="cpu", weights_only=True)
        priors = payload.get("class_priors")
        centroid = payload.get("centroid")
        return cls(
            categories=tuple(payload["categories"]),
            embeddings=payload["embeddings"].to(device),
            logit_scale=float(payload["logit_scale"]),
            logit_bias=float(payload["logit_bias"]),
            class_priors=priors.to(device) if priors is not None else None,
            centroid=centroid.to(device) if centroid is not None else None,
        )

    @classmethod
    def from_teacher(
        cls,
        teacher: SiglipTeacher,
        categories: Sequence[str],
        templates: Sequence[str] | None = None,
        synonyms: dict[str, Sequence[str]] | None = None,
    ) -> VocabularyBank:
        """Build an authoritative vocabulary bank from a live SigLIP teacher model."""
        category_list = tuple(categories)
        templates_list = templates if templates is not None else DEFAULT_PROMPT_TEMPLATES
        synonyms_dict = synonyms if synonyms is not None else DEFAULT_CATEGORY_SYNONYMS
        bank_tensor, meta = teacher.build_text_bank(
            category_list,
            templates=templates_list,
            synonyms=synonyms_dict,
        )
        return cls(
            categories=category_list,
            embeddings=bank_tensor.to(torch.float32),
            logit_scale=meta["teacher"]["logit_scale"],
            logit_bias=meta["teacher"]["logit_bias"],
        )


def fit_temperature(logits: Tensor, targets: Tensor, max_iter: int = 100) -> float:
    """Fit a single softmax temperature by minimizing NLL (Guo et al., 2017).

    /// @brief Post-hoc calibration: after fitting, max softmax prob ~= P(prediction is correct).
    /// @param logits Float tensor [N, C] of uncalibrated student logits.
    /// @param targets Long tensor [N] of reference labels (e.g. teacher top-1).
    /// @param max_iter LBFGS iteration cap.
    /// @return Fitted temperature T > 0 (divide logits by T).
    """
    logits = logits.detach().float()
    targets = targets.detach().long().to(logits.device)
    log_t = torch.zeros(1, device=logits.device, requires_grad=True)
    optimizer = torch.optim.LBFGS([log_t], lr=0.1, max_iter=max_iter)

    def closure() -> Tensor:
        optimizer.zero_grad()
        loss = F.cross_entropy(logits / log_t.exp(), targets)
        loss.backward()
        return loss

    optimizer.step(closure)
    return float(log_t.exp().item())


def expected_calibration_error(probs: Tensor, targets: Tensor, n_bins: int = 15) -> float:
    """Compute top-1 expected calibration error.

    /// @brief Bin-weighted gap between confidence and accuracy.
    /// @param probs Float tensor [N, C] of class probabilities.
    /// @param targets Long tensor [N] of reference labels.
    /// @param n_bins Number of equal-width confidence bins.
    /// @return ECE in [0, 1].
    """
    conf, pred = probs.max(dim=-1)
    correct = (pred == targets.to(pred.device)).float()
    edges = torch.linspace(0.0, 1.0, n_bins + 1, device=probs.device)
    ece = torch.zeros((), device=probs.device)
    for lo, hi in zip(edges[:-1], edges[1:], strict=True):
        in_bin = (conf > lo) & (conf <= hi)
        if in_bin.any():
            ece += in_bin.float().mean() * (conf[in_bin].mean() - correct[in_bin].mean()).abs()
    return float(ece.item())


__all__ = [
    "DEFAULT_CATEGORY_SYNONYMS",
    "VocabularyBank",
    "expected_calibration_error",
    "fit_temperature",
]
