"""SigLIP teacher adapter for open-vocabulary distillation and baselines."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

from PIL import Image
import torch
from torch import Tensor
from transformers import AutoProcessor, SiglipModel

SIGLIP_MODEL_NAME: str = "google/siglip-base-patch16-224"
SIGLIP_REVISION: str = "7fd15f0689c79d79e38b1c2e2e2370a7bf2761ed"
SIGLIP_EMBEDDING_DIM: int = 768
SIGLIP_INPUT_SIZE: tuple[int, int] = (224, 224)

DEFAULT_PROMPT_TEMPLATES: tuple[str, ...] = (
    "a photo of a {category}",
    "a cropped image of a {category}",
    "a {category} on the road",
    "a {category} in a traffic scene",
)


@dataclass(frozen=True)
class SiglipMetadata:
    """Immutable metadata contract for the pinned SigLIP teacher."""

    model_name: str
    revision: str
    embedding_dim: int
    input_size: tuple[int, int]
    logit_scale: float
    logit_bias: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "model_name": self.model_name,
            "revision": self.revision,
            "embedding_dim": self.embedding_dim,
            "input_size": list(self.input_size),
            "logit_scale": self.logit_scale,
            "logit_bias": self.logit_bias,
        }


class SiglipTeacher:
    """Frozen SigLIP teacher model for text and crop representation.

    /// @brief Frozen teacher adapter wrapping SigLIP for vision-language alignment.
    /// @param model_name Pretrained model identifier or local path.
    /// @param revision Immutable git revision / commit SHA.
    /// @param device Execution device for inference.
    """

    def __init__(
        self,
        model_name: str = SIGLIP_MODEL_NAME,
        revision: str = SIGLIP_REVISION,
        device: torch.device | str = "cpu",
    ) -> None:
        self.model_name = model_name
        self.revision = revision
        self.device = torch.device(device)

        self.model: SiglipModel = SiglipModel.from_pretrained(
            model_name, revision=revision
        )
        self.model.eval()
        for param in self.model.parameters():
            param.requires_grad = False
        self.model.to(self.device)

        self.processor: AutoProcessor = AutoProcessor.from_pretrained(
            model_name, revision=revision
        )

        scale_val = float(self.model.logit_scale.detach().cpu().item())
        bias_val = float(self.model.logit_bias.detach().cpu().item())
        self._metadata = SiglipMetadata(
            model_name=model_name,
            revision=revision,
            embedding_dim=SIGLIP_EMBEDDING_DIM,
            input_size=SIGLIP_INPUT_SIZE,
            logit_scale=scale_val,
            logit_bias=bias_val,
        )

    @property
    def metadata(self) -> SiglipMetadata:
        """Return the immutable teacher metadata."""
        return self._metadata

    @torch.no_grad()
    def encode_text(
        self,
        texts: Sequence[str],
        batch_size: int = 64,
    ) -> Tensor:
        """Encode textual prompts into unit-normalized L2 embeddings.

        /// @param texts Sequence of text strings to encode.
        /// @param batch_size Batch size for memory bounding.
        /// @return Float32 tensor of shape [N, D] with unit L2 norm.
        """
        if not texts:
            return torch.empty((0, SIGLIP_EMBEDDING_DIM), dtype=torch.float32, device="cpu")

        embeddings_list: list[Tensor] = []
        for start_idx in range(0, len(texts), batch_size):
            batch_texts = list(texts[start_idx : start_idx + batch_size])
            inputs = self.processor(
                text=batch_texts,
                padding="max_length",
                return_tensors="pt",
            )
            input_ids = inputs["input_ids"].to(self.device)
            attention_mask = inputs.get("attention_mask")
            if attention_mask is not None:
                attention_mask = attention_mask.to(self.device)

            output = self.model.get_text_features(
                input_ids=input_ids,
                attention_mask=attention_mask,
            )
            features = (
                output.pooler_output
                if hasattr(output, "pooler_output") and output.pooler_output is not None
                else output[0]
            )
            normalized = torch.nn.functional.normalize(features.to(torch.float32), dim=-1)
            embeddings_list.append(normalized.cpu())

        return torch.cat(embeddings_list, dim=0)

    @torch.no_grad()
    def encode_images(
        self,
        images: Sequence[Image.Image] | Tensor,
        batch_size: int = 32,
    ) -> Tensor:
        """Encode image crops into unit-normalized L2 embeddings.

        /// @param images Sequence of PIL Images or pre-batched image tensor.
        /// @param batch_size Batch size for bounded memory throughput.
        /// @return Float32 tensor of shape [N, D] with unit L2 norm.
        """
        if len(images) == 0:
            return torch.empty((0, SIGLIP_EMBEDDING_DIM), dtype=torch.float32, device="cpu")

        embeddings_list: list[Tensor] = []
        if isinstance(images, Tensor):
            for start_idx in range(0, images.shape[0], batch_size):
                batch_tensor = images[start_idx : start_idx + batch_size].to(self.device)
                output = self.model.get_image_features(pixel_values=batch_tensor)
                features = (
                    output.pooler_output
                    if hasattr(output, "pooler_output") and output.pooler_output is not None
                    else output[0]
                )
                normalized = torch.nn.functional.normalize(features.to(torch.float32), dim=-1)
                embeddings_list.append(normalized.cpu())
        else:
            for start_idx in range(0, len(images), batch_size):
                batch_imgs = [img.convert("RGB") for img in images[start_idx : start_idx + batch_size]]
                inputs = self.processor(
                    images=batch_imgs,
                    return_tensors="pt",
                )
                pixel_values = inputs["pixel_values"].to(self.device)
                output = self.model.get_image_features(pixel_values=pixel_values)
                features = (
                    output.pooler_output
                    if hasattr(output, "pooler_output") and output.pooler_output is not None
                    else output[0]
                )
                normalized = torch.nn.functional.normalize(features.to(torch.float32), dim=-1)
                embeddings_list.append(normalized.cpu())

        return torch.cat(embeddings_list, dim=0)

    def build_text_bank(
        self,
        categories: Sequence[str],
        templates: Sequence[str] | None = None,
    ) -> tuple[Tensor, dict[str, Any]]:
        """Construct an averaged, unit-normalized text representation bank for categories.

        /// @param categories Ordered list of category names.
        /// @param templates Formatting strings containing '{category}'.
        /// @return Tuple of (normalized bank [C, D], metadata dictionary).
        """
        prompt_templates = tuple(templates) if templates is not None else DEFAULT_PROMPT_TEMPLATES
        category_list = list(categories)

        category_embeddings: list[Tensor] = []
        for cat in category_list:
            formatted_prompts = [tmpl.format(category=cat.replace("_", " ")) for tmpl in prompt_templates]
            prompt_embeds = self.encode_text(formatted_prompts)
            mean_embed = prompt_embeds.mean(dim=0, keepdim=True)
            norm_embed = torch.nn.functional.normalize(mean_embed, dim=-1)
            category_embeddings.append(norm_embed)

        bank = torch.cat(category_embeddings, dim=0) if category_embeddings else torch.empty((0, SIGLIP_EMBEDDING_DIM))
        metadata = {
            "categories": category_list,
            "prompt_templates": list(prompt_templates),
            "teacher": self._metadata.to_dict(),
            "bank_shape": list(bank.shape),
        }
        return bank, metadata

    def compute_similarity(
        self,
        image_embeddings: Tensor,
        text_bank: Tensor,
    ) -> tuple[Tensor, Tensor]:
        """Compute calibrated logit similarity and sigmoid probabilities.

        /// @param image_embeddings [N, D] unit-normalized image vectors.
        /// @param text_bank [C, D] unit-normalized category vectors.
        /// @return Tuple of (logits [N, C], probabilities [N, C]).
        """
        if image_embeddings.numel() == 0 or text_bank.numel() == 0:
            empty = torch.empty((image_embeddings.shape[0], text_bank.shape[0]), dtype=torch.float32)
            return empty, empty

        scale = torch.exp(torch.tensor(self._metadata.logit_scale, dtype=torch.float32))
        bias = torch.tensor(self._metadata.logit_bias, dtype=torch.float32)

        cosine_sim = torch.matmul(image_embeddings.to(torch.float32), text_bank.to(torch.float32).t())
        logits = cosine_sim * scale + bias
        probabilities = torch.sigmoid(logits)
        return logits, probabilities


__all__ = [
    "DEFAULT_PROMPT_TEMPLATES",
    "SIGLIP_EMBEDDING_DIM",
    "SIGLIP_INPUT_SIZE",
    "SIGLIP_MODEL_NAME",
    "SIGLIP_REVISION",
    "SiglipMetadata",
    "SiglipTeacher",
]
