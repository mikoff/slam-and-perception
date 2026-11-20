"""End-to-end open-vocabulary semantic heads (Option A: DenseAnchor, Option B: RoIPooled)."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Any

import torch
from torch import Tensor, nn
import torch.nn.functional as F
from torchvision.ops import MultiScaleRoIAlign, box_iou

from .losses import sigmoid_focal_loss
from .neck import DepthwiseSeparableConv
from .quad_geometry import pairwise_quad_iou, quad_area
from .region_head import RegionEmbeddingHead


def _default_logit_bias(prior_prob: float = 0.008) -> float:
    return -math.log((1.0 - prior_prob) / prior_prob)


class _ContrastiveClassifierMixin(nn.Module):
    """Shared L2-normalized SigLIP text-bank projection utilities (zero runtime text encoder)."""

    log_scale: nn.Parameter
    logit_bias: nn.Parameter
    distractor_embeddings: Tensor | None
    visual_centroid: Tensor | None

    def _init_contrastive_scale(
        self,
        init_scale: float = 80.0,
        prior_prob: float = 0.008,
    ) -> None:
        self.log_scale = nn.Parameter(
            torch.tensor(math.log(init_scale), dtype=torch.float32)
        )
        self.logit_bias = nn.Parameter(
            torch.tensor(_default_logit_bias(prior_prob), dtype=torch.float32)
        )
        self.register_buffer("distractor_embeddings", None, persistent=True)
        self.register_buffer("visual_centroid", None, persistent=True)

    def set_distractor_embeddings(self, distractor_embeddings: Tensor | None) -> None:
        """Register unit-normalized L2 distractor text embeddings for background veto."""
        if distractor_embeddings is None or distractor_embeddings.numel() == 0:
            self.distractor_embeddings = None
            return
        ref_device = self.log_scale.device
        self.distractor_embeddings = F.normalize(
            distractor_embeddings.detach().to(device=ref_device, dtype=torch.float32),
            p=2.0,
            dim=-1,
        )

    def set_visual_centroid(self, visual_centroid: Tensor | None) -> None:
        """Register empirical background visual centroid [1, D] for prompt de-biasing."""
        if visual_centroid is None or visual_centroid.numel() == 0:
            self.visual_centroid = None
            return
        ref_device = self.log_scale.device
        c_vec = visual_centroid.detach().to(device=ref_device, dtype=torch.float32)
        if c_vec.ndim == 1:
            c_vec = c_vec.unsqueeze(0)
        self.visual_centroid = c_vec

    def compute_prompt_bias(self, txt_f: Tensor) -> Tensor:
        """Compute damped clamped positive baseline cosine offset [1, C] from visual_centroid."""
        c_vec: Tensor | None = getattr(self, "visual_centroid", None)
        if c_vec is None or c_vec.numel() == 0:
            return txt_f.new_zeros((1, txt_f.shape[0]))
        c_f = c_vec.to(device=txt_f.device, dtype=torch.float32)
        raw = (c_f @ txt_f.t()).clamp(min=0.0)
        return torch.where(raw <= 0.05, 0.85 * raw.clamp(max=0.016), raw)

    def _load_from_state_dict(
        self,
        state_dict: dict[str, Any],
        prefix: str,
        local_metadata: dict[str, Any],
        strict: bool,
        missing_keys: list[str],
        unexpected_keys: list[str],
        error_msgs: list[str],
    ) -> None:
        ref_device = self.log_scale.device
        dist_key = prefix + "distractor_embeddings"
        saved_dist = getattr(self, "distractor_embeddings", None)
        if dist_key in state_dict and isinstance(state_dict[dist_key], Tensor):
            self.distractor_embeddings = F.normalize(
                state_dict[dist_key].detach().to(device=ref_device, dtype=torch.float32),
                p=2.0,
                dim=-1,
            )
            saved_dist = self.distractor_embeddings
        elif dist_key not in state_dict and saved_dist is not None:
            self.distractor_embeddings = None

        cent_key = prefix + "visual_centroid"
        saved_cent = getattr(self, "visual_centroid", None)
        if cent_key in state_dict and isinstance(state_dict[cent_key], Tensor):
            c_vec = state_dict[cent_key].detach().to(device=ref_device, dtype=torch.float32)
            self.visual_centroid = c_vec.unsqueeze(0) if c_vec.ndim == 1 else c_vec
            saved_cent = self.visual_centroid
        elif cent_key not in state_dict and saved_cent is not None:
            self.visual_centroid = None

        super()._load_from_state_dict(
            state_dict,
            prefix,
            local_metadata,
            strict,
            missing_keys,
            unexpected_keys,
            error_msgs,
        )
        if dist_key not in state_dict and saved_dist is not None:
            self.distractor_embeddings = saved_dist
        if cent_key not in state_dict and saved_cent is not None:
            self.visual_centroid = saved_cent

    def effective_scale(self) -> Tensor:
        """Return clamped positive temperature scale aligned with SigLIP's operating range."""
        return self.log_scale.float().clamp(min=math.log(5.0), max=math.log(125.0)).exp()

    def classify_embeddings(
        self,
        visual_embeddings: Tensor,
        text_embeddings: Tensor,
        *,
        apply_calibration: bool = True,
    ) -> tuple[Tensor, Tensor]:
        """Score unit visual embeddings [N, D] against unit text bank [C, D]."""
        if visual_embeddings.numel() == 0 or text_embeddings.numel() == 0:
            num_n = visual_embeddings.shape[0]
            num_c = text_embeddings.shape[0]
            empty = visual_embeddings.new_empty((num_n, num_c), dtype=torch.float32)
            return empty, empty
        vis_f = F.normalize(visual_embeddings.float(), p=2.0, dim=-1)
        txt_f = F.normalize(
            text_embeddings.to(device=vis_f.device, dtype=torch.float32), p=2.0, dim=-1
        )
        scale = self.effective_scale()
        cos_fg = (vis_f @ txt_f.t()) - self.compute_prompt_bias(txt_f)
        if apply_calibration and cos_fg.shape[-1] > 2:
            med = cos_fg.median(dim=-1, keepdim=True).values
            cos_fg = cos_fg - torch.where(med < 0.0, 0.75 * med, 0.36 * med)
        logits = scale * cos_fg + self.logit_bias.float()
        if apply_calibration:
            dist_emb: Tensor | None = getattr(self, "distractor_embeddings", None)
            if dist_emb is not None and dist_emb.numel() > 0:
                dist_f = F.normalize(
                    dist_emb.to(device=vis_f.device, dtype=torch.float32), p=2.0, dim=-1
                )
                max_dist_cos = (
                    ((vis_f @ dist_f.t()) - self.compute_prompt_bias(dist_f))
                    .max(dim=-1, keepdim=True)
                    .values
                )
                logits = logits - 2.0 * scale * F.relu(max_dist_cos - cos_fg + 0.004)
            if cos_fg.shape[-1] > 1:
                cos_top1 = cos_fg.max(dim=-1, keepdim=True).values
                logits = logits - 0.5 * scale * F.relu(cos_top1 - cos_fg)
        return logits, torch.sigmoid(logits)


class DenseAnchorSemanticHead(_ContrastiveClassifierMixin):
    """Option A: YOLO-World style single-stage dense per-anchor open-vocabulary head.

    Uses a shared Conv2d semantic tower across P2-P5 pyramid levels with zero RoIAlign ops.
    Supports both full dense Conv2d execution (for static ONNX/TFLite/NPU graph export)
    and bit-exact sparse indexed projection (for fast training and top-K inference).
    """

    def __init__(
        self,
        in_channels: int = 128,
        hidden_channels: int = 128,
        embedding_dim: int = 768,
        init_scale: float = 80.0,
        prior_prob: float = 0.008,
    ) -> None:
        super().__init__()
        self.in_channels = in_channels
        self.hidden_channels = hidden_channels
        self.embedding_dim = embedding_dim
        self.tower = nn.Sequential(
            DepthwiseSeparableConv(in_channels, hidden_channels),
            DepthwiseSeparableConv(hidden_channels, hidden_channels),
        )
        self.proj = nn.Conv2d(hidden_channels, embedding_dim, kernel_size=1, bias=True)
        self._init_contrastive_scale(init_scale=init_scale, prior_prob=prior_prob)

    def get_config(self) -> dict[str, Any]:
        """Return constructor configuration for checkpoint serialization."""
        return {
            "in_channels": self.in_channels,
            "hidden_channels": self.hidden_channels,
            "embedding_dim": self.embedding_dim,
        }

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> DenseAnchorSemanticHead:
        """Instantiate DenseAnchorSemanticHead from serialized config."""
        return cls(**config)

    def forward_tower(self, features: tuple[Tensor, ...]) -> tuple[Tensor, ...]:
        """Run shared semantic depthwise-separable tower across all pyramid levels."""
        return tuple(self.tower(feat) for feat in features)

    def forward_dense(self, features: tuple[Tensor, ...]) -> tuple[Tensor, ...]:
        """Compute dense L2-normalized [B, D, H_l, W_l] embeddings via pure Conv2d ops."""
        tower_feats = self.forward_tower(features)
        return tuple(
            F.normalize(self.proj(feat).float(), p=2.0, dim=1) for feat in tower_feats
        )

    def forward_indexed(
        self,
        features: tuple[Tensor, ...],
        batch_indices: Tensor,
        flat_anchor_indices: Tensor,
        *,
        tower_features: tuple[Tensor, ...] | None = None,
    ) -> Tensor:
        """Project only selected (batch_idx, flat_anchor_idx) locations, bit-exact with forward_dense."""
        if batch_indices.numel() == 0:
            return features[0].new_empty((0, self.embedding_dim), dtype=torch.float32)
        t_feats = tower_features if tower_features is not None else self.forward_tower(features)
        flat_tower = torch.cat(
            [
                feat.permute(0, 2, 3, 1).reshape(feat.shape[0], -1, self.hidden_channels)
                for feat in t_feats
            ],
            dim=1,
        )
        gathered = flat_tower[batch_indices, flat_anchor_indices]
        weight_2d = self.proj.weight.squeeze(-1).squeeze(-1)
        projected = F.linear(gathered, weight_2d, self.proj.bias)
        return F.normalize(projected.float(), p=2.0, dim=-1)


class RoIPooledSemanticHead(_ContrastiveClassifierMixin):
    """Option B: Detic/ViLD style multi-scale focal + context RoI open-vocabulary head.

    Pools interior + contextual PAN features via MultiScaleRoIAlign without any
    [xc/W, yc/H] box-coordinate shortcut, preventing vertical-position bias.
    """

    def __init__(
        self,
        in_channels: int = 128,
        roi_channels: int = 96,
        embedding_dim: int = 768,
        output_size: int = 5,
        hidden_dim: int = 512,
        strides: tuple[int, ...] = (4, 8, 16, 32),
        focal_margin: float = 0.15,
        context_margin: float = 0.45,
        dropout: float = 0.1,
        init_scale: float = 80.0,
        prior_prob: float = 0.008,
    ) -> None:
        super().__init__()
        self.in_channels = in_channels
        self.roi_channels = roi_channels
        self.embedding_dim = embedding_dim
        self.output_size = output_size
        self.hidden_dim = hidden_dim
        self.strides = strides
        self.focal_margin = focal_margin
        self.context_margin = context_margin
        self.dropout = dropout

        self.semantic_adapter = DepthwiseSeparableConv(in_channels, roi_channels)
        self.featmap_names = [f"p{i}" for i in range(len(strides))]
        self.roi_align = MultiScaleRoIAlign(
            featmap_names=self.featmap_names,
            output_size=output_size,
            sampling_ratio=2,
        )
        visual_dim = 2 * roi_channels * output_size * output_size
        self.projector = nn.Sequential(
            nn.LayerNorm(visual_dim),
            nn.Linear(visual_dim, hidden_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, embedding_dim),
        )
        self._init_contrastive_scale(init_scale=init_scale, prior_prob=prior_prob)

    def get_config(self) -> dict[str, Any]:
        """Return constructor configuration for checkpoint serialization."""
        return {
            "in_channels": self.in_channels,
            "roi_channels": self.roi_channels,
            "embedding_dim": self.embedding_dim,
            "output_size": self.output_size,
            "hidden_dim": self.hidden_dim,
            "strides": self.strides,
            "focal_margin": self.focal_margin,
            "context_margin": self.context_margin,
            "dropout": self.dropout,
        }

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> RoIPooledSemanticHead:
        """Instantiate RoIPooledSemanticHead from serialized config."""
        cfg = dict(config)
        cfg["strides"] = tuple(cfg["strides"])
        return cls(**cfg)

    def forward(
        self,
        features: tuple[Tensor, ...] | list[Tensor],
        boxes: list[Tensor] | Tensor,
        image_shapes: list[tuple[int, int]] | None = None,
    ) -> Tensor:
        """Pool focal + context RoI features across pyramid levels and project to unit L2 sphere."""
        if len(features) != len(self.featmap_names):
            raise ValueError(
                f"Expected {len(self.featmap_names)} feature levels, got {len(features)}"
            )
        if isinstance(boxes, Tensor):
            if boxes.ndim == 2:
                box_list = [boxes]
            elif boxes.ndim == 3:
                box_list = [b for b in boxes]
            else:
                raise ValueError(f"Unsupported box tensor shape: {boxes.shape}")
        else:
            box_list = list(boxes)

        total_boxes = sum(b.shape[0] for b in box_list)
        if total_boxes == 0:
            return features[0].new_empty((0, self.embedding_dim), dtype=torch.float32)

        if image_shapes is None:
            h0, w0 = features[0].shape[-2] * self.strides[0], features[0].shape[-1] * self.strides[0]
            image_shapes = [(h0, w0)] * len(box_list)

        adapted = [self.semantic_adapter(feat) for feat in features]
        feat_dict = OrderedDict(
            (name, feat) for name, feat in zip(self.featmap_names, adapted, strict=True)
        )
        focal_boxes = [
            RegionEmbeddingHead.expand_boxes(b.float(), self.focal_margin, shape)
            for b, shape in zip(box_list, image_shapes, strict=True)
        ]
        context_boxes = [
            RegionEmbeddingHead.expand_boxes(b.float(), self.context_margin, shape)
            for b, shape in zip(box_list, image_shapes, strict=True)
        ]
        pooled_focal = self.roi_align(feat_dict, focal_boxes, image_shapes=image_shapes).flatten(1)
        pooled_context = self.roi_align(
            feat_dict, context_boxes, image_shapes=image_shapes
        ).flatten(1)
        fused = torch.cat((pooled_focal, pooled_context), dim=-1)
        projected = self.projector(fused)
        return F.normalize(projected.float(), p=2.0, dim=-1)


@dataclass(frozen=True)
class OpenVocabLossOutput:
    """Decomposed open-vocabulary SigLIP-primary distillation + human-GT support loss."""

    total: Tensor
    focal: Tensor
    teacher_kl: Tensor
    teacher_cosine: Tensor
    num_foreground: int
    num_background: int
    num_oov: int


class OpenVocabularySigmoidLoss(nn.Module):
    """SigLIP-primary manifold distillation with lightweight human-GT support.

    Label convention:
      - ``0 <= label < C``: in-vocabulary foreground class (supports SigLIP distillation
        plus compatible human-GT tie-breaker margin and soft teacher-guided support).
      - ``label == -1``: background / hard-negative clutter (light focal regularization on
        annotated classes + pull toward distractor embeddings when available).
      - ``label == -2``: out-of-vocabulary (OOV) or unlabelled SAM discovery region
        (supervised exclusively via SigLIP teacher distillation).
    """

    def __init__(
        self,
        *,
        focal_alpha: float = 0.25,
        focal_gamma: float = 2.0,
        kl_weight: float = 1.50,
        in_vocab_cosine_weight: float = 1.50,
        oov_cosine_weight: float = 1.50,
        infonce_weight: float = 0.30,
        gt_support_weight: float = 0.35,
        contrastive_kd_tau: float = 0.025,
        class_counts: Tensor | None = None,
        seesaw_p: float = 0.25,
        unannotated_fg_weight: float = 0.08,
    ) -> None:
        super().__init__()
        self.focal_alpha = focal_alpha
        self.focal_gamma = focal_gamma
        self.kl_weight = kl_weight
        self.in_vocab_cosine_weight = in_vocab_cosine_weight
        self.oov_cosine_weight = oov_cosine_weight
        self.infonce_weight = infonce_weight
        self.gt_support_weight = gt_support_weight
        self.contrastive_kd_tau = contrastive_kd_tau
        self.seesaw_p = seesaw_p
        self.unannotated_fg_weight = unannotated_fg_weight
        self.register_buffer(
            "class_counts",
            class_counts.float().clone() if class_counts is not None else None,
            persistent=False,
        )

    def forward(
        self,
        head: DenseAnchorSemanticHead | RoIPooledSemanticHead,
        visual_embeddings: Tensor,
        text_embeddings: Tensor,
        labels: Tensor,
        *,
        teacher_embeddings: Tensor | None = None,
        has_teacher: Tensor | None = None,
        annotated_mask: Tensor | None = None,
    ) -> OpenVocabLossOutput:
        """Compute SigLIP-primary distillation + human-GT support loss for Option A or Option B."""
        device = visual_embeddings.device
        zero = visual_embeddings.sum() * 0.0
        if visual_embeddings.numel() == 0:
            return OpenVocabLossOutput(zero, zero, zero, zero, 0, 0, 0)

        num_classes = text_embeddings.shape[0]
        vis_norm = F.normalize(visual_embeddings.float(), p=2.0, dim=-1)
        txt_norm = F.normalize(
            text_embeddings.to(device=device, dtype=torch.float32), p=2.0, dim=-1
        )
        logits_raw, _ = head.classify_embeddings(
            vis_norm, txt_norm, apply_calibration=False
        )
        logits = logits_raw.float()
        scale_det = head.effective_scale().detach().clamp(min=1.0)

        fg_mask = (labels >= 0) & (labels < num_classes)
        bg_mask = labels == -1
        oov_mask = labels == -2

        num_fg = int(fg_mask.sum().item())
        num_bg = int(bg_mask.sum().item())
        num_oov = int(oov_mask.sum().item())

        dist_emb_raw: Tensor | None = getattr(head, "distractor_embeddings", None)
        dist_norm = (
            F.normalize(dist_emb_raw.to(device=device, dtype=torch.float32), p=2.0, dim=-1)
            if (dist_emb_raw is not None and dist_emb_raw.numel() > 0)
            else None
        )

        # --- 1. Primary Loss: Unfiltered SigLIP Teacher Distillation ---
        kl_loss = zero
        cosine_loss = zero
        infonce_loss = zero
        if teacher_embeddings is not None and has_teacher is not None:
            valid_t = has_teacher.to(device=device, dtype=torch.bool) & (fg_mask | oov_mask)
            if valid_t.any():
                z_s = vis_norm[valid_t]
                z_t = F.normalize(
                    teacher_embeddings[valid_t].to(device=device, dtype=torch.float32),
                    p=2.0,
                    dim=-1,
                )
                # Direct 768-D unit-sphere alignment across ALL regions (no Top-5 gating)
                cos_err = 1.0 - (z_s * z_t).sum(dim=-1)
                cos_weights = torch.where(
                    oov_mask[valid_t],
                    torch.full_like(cos_err, self.oov_cosine_weight),
                    torch.full_like(cos_err, self.in_vocab_cosine_weight),
                )
                cosine_loss = (cos_err * cos_weights).mean()

                # Relational vocabulary KL over 37 foreground + distractor text bank
                full_txt = (
                    torch.cat([txt_norm, dist_norm], dim=0)
                    if dist_norm is not None
                    else txt_norm
                )
                full_bias = head.compute_prompt_bias(full_txt)
                s_full_cos = (z_s @ full_txt.t()).float() - full_bias
                t_full_cos = (z_t @ full_txt.t()).float() - full_bias
                s_kd = s_full_cos / self.contrastive_kd_tau
                t_kd = t_full_cos / self.contrastive_kd_tau
                kl_loss = (
                    F.kl_div(
                        F.log_softmax(s_kd, dim=-1),
                        F.softmax(t_kd, dim=-1),
                        reduction="batchmean",
                    )
                    * (self.contrastive_kd_tau * 40.0)
                )
                # Direct text-subspace cosine alignment so student matches teacher on 37+7 axes
                subspace_mse = ((s_full_cos - t_full_cos) * 14.0).pow(2).mean()
                mean_cos_loss = (
                    (s_full_cos.mean(dim=-1) - t_full_cos.mean(dim=-1)) * 16.0
                ).pow(2).mean()
                # Teacher Top-1 margin loss on high-confidence foreground crops/discoveries
                t_fg_all = t_full_cos[:, :num_classes]
                s_fg_all = s_full_cos[:, :num_classes]
                if num_classes > 1:
                    t_top2_vals, t_top2_idxs = t_fg_all.topk(k=2, dim=-1)
                    t_conf_mask = (t_top2_vals[:, 0] >= 0.045) & (
                        (t_top2_vals[:, 0] - t_top2_vals[:, 1]) >= 0.020
                    )
                    if t_conf_mask.any():
                        t_win = t_top2_idxs[t_conf_mask, 0]
                        s_conf = s_fg_all[t_conf_mask]
                        s_win = s_conf.gather(1, t_win.unsqueeze(1)).squeeze(1)
                        s_neg = s_conf.masked_fill(
                            F.one_hot(t_win, num_classes=num_classes).bool(), -1e4
                        ).max(dim=-1).values
                        t_margin_loss = 2.5 * F.relu(0.030 - (s_win - s_neg)).mean()
                    else:
                        t_margin_loss = zero
                else:
                    t_margin_loss = zero
                cosine_loss = (
                    cosine_loss + 0.30 * subspace_mse + 0.30 * mean_cos_loss + t_margin_loss
                )

                # In-batch contrastive InfoNCE (mask off-diagonal duplicate teacher crops)
                num_valid_t = int(z_s.shape[0])
                if num_valid_t >= 2 and self.infonce_weight > 0.0:
                    t_self = z_t @ z_t.t()
                    eye = torch.eye(num_valid_t, dtype=torch.bool, device=device)
                    dup_mask = (t_self > 0.995) & (~eye)
                    sim_st = (z_s @ z_t.t()).float() / 0.07
                    sim_st = sim_st.masked_fill(dup_mask, -1e4)
                    infonce_loss = F.cross_entropy(
                        sim_st, torch.arange(num_valid_t, device=device)
                    )

        # --- 2. Secondary Support: Compatible Human-GT Margin & Soft Teacher Support ---
        support_loss = zero
        fg_bias = head.compute_prompt_bias(txt_norm)
        counts_buf: Tensor | None = getattr(self, "class_counts", None)
        if fg_mask.any():
            fg_idx = fg_mask.nonzero(as_tuple=False).squeeze(-1)
            y_fg = labels[fg_idx]
            s_fg_cos = (vis_norm[fg_idx] @ txt_norm.t()).float() - fg_bias

            has_t_fg = (
                has_teacher[fg_idx].to(device=device, dtype=torch.bool)
                if (teacher_embeddings is not None and has_teacher is not None)
                else torch.zeros(y_fg.shape[0], dtype=torch.bool, device=device)
            )
            if has_t_fg.any():
                assert teacher_embeddings is not None
                z_t_fg = F.normalize(
                    teacher_embeddings[fg_idx].to(device=device, dtype=torch.float32),
                    p=2.0,
                    dim=-1,
                )
                t_fg_cos = (z_t_fg @ txt_norm.t()).float() - fg_bias
                gt_cos_t = t_fg_cos.gather(1, y_fg.unsqueeze(1)).squeeze(1)
                max_cos_t = t_fg_cos.max(dim=-1).values
                compat = (~has_t_fg) | (gt_cos_t >= (max_cos_t - 0.025))
            else:
                t_fg_cos = s_fg_cos.detach()
                compat = torch.ones(y_fg.shape[0], dtype=torch.bool, device=device)

            if compat.any():
                c_rows = compat.nonzero(as_tuple=False).squeeze(-1)
                y_c = y_fg[c_rows]
                s_c = s_fg_cos[c_rows]
                logits_c = logits[fg_idx[c_rows]]

                # 2a. Positive Cosine Margin Support inside SigLIP's natural margin
                s_pos = s_c.gather(1, y_c.unsqueeze(1)).squeeze(1)
                one_hot_c = F.one_hot(y_c, num_classes=num_classes).to(dtype=torch.float32)
                if num_classes > 1:
                    s_hardest_neg = s_c.masked_fill(one_hot_c.bool(), -1e4).max(dim=-1).values
                    margin_loss = F.relu(0.040 - (s_pos - s_hardest_neg)).mean()
                else:
                    margin_loss = zero

                # 2b. Soft Teacher-Guided Sigmoid Support
                has_t_c = has_t_fg[c_rows]
                if has_t_c.any():
                    t_probs_c = torch.sigmoid(90.0 * t_fg_cos[c_rows] - 4.40).detach()
                    soft_neg = (0.50 * t_probs_c).clamp(max=0.25)
                    soft_tgt = torch.where(
                        has_t_c.unsqueeze(1),
                        torch.maximum(soft_neg, one_hot_c),
                        one_hot_c,
                    )
                else:
                    soft_tgt = one_hot_c

                entry_w = torch.ones_like(soft_tgt)
                if annotated_mask is not None:
                    ann_c = annotated_mask[fg_idx[c_rows]].to(device=device, dtype=torch.bool)
                    ann_c = ann_c.clone()
                    ann_c.scatter_(1, y_c.unsqueeze(1), True)
                    entry_w = torch.where(
                        ann_c,
                        entry_w,
                        torch.full_like(entry_w, self.unannotated_fg_weight),
                    )
                if counts_buf is not None and counts_buf.numel() == num_classes:
                    cnts = counts_buf.to(device=device, dtype=torch.float32).clamp(min=10.0)
                    ref_cnt = cnts.median().clamp(min=100.0)
                    pos_boost = (ref_cnt / cnts).pow(0.15).clamp(min=1.0, max=1.5)
                    n_i = cnts[y_c].unsqueeze(1)
                    n_j = cnts.unsqueeze(0)
                    mitigation = (n_j / n_i).pow(self.seesaw_p).clamp(max=1.0)
                    both_rare = (n_i < 1500.0) & (n_j < 1500.0)
                    mitigation = torch.where(both_rare, mitigation.clamp(min=0.75), mitigation)
                    entry_w = entry_w * mitigation
                    entry_w.scatter_(1, y_c.unsqueeze(1), pos_boost[y_c].unsqueeze(1))

                per_entry_fg = F.binary_cross_entropy_with_logits(
                    logits_c, soft_tgt, reduction="none"
                )
                soft_bce_loss = (per_entry_fg * entry_w).sum() / (
                    float(max(y_c.numel(), 1)) * scale_det
                )
                support_loss = support_loss + self.gt_support_weight * (
                    soft_bce_loss + 1.5 * margin_loss
                )

        # --- 3. Light Mined-Background Regularization + Distractor Pull ---
        if bg_mask.any():
            bg_idx = bg_mask.nonzero(as_tuple=False).squeeze(-1)
            bg_logits = logits[bg_idx]
            bg_focal = sigmoid_focal_loss(
                bg_logits,
                torch.zeros_like(bg_logits),
                alpha=self.focal_alpha,
                gamma=self.focal_gamma,
            )
            bg_w = torch.ones_like(bg_logits)
            if annotated_mask is not None:
                ann_bg = annotated_mask[bg_idx].to(device=device, dtype=torch.bool)
                bg_w = torch.where(ann_bg, bg_w, torch.full_like(bg_w, 0.35))
            if counts_buf is not None and counts_buf.numel() == num_classes:
                cnts = counts_buf.to(device=device, dtype=torch.float32).clamp(min=10.0)
                ref_cnt = cnts.median().clamp(min=100.0)
                bg_freq_w = (cnts / ref_cnt).pow(0.15).clamp(min=0.70, max=1.0)
                bg_w = bg_w * bg_freq_w.unsqueeze(0)
            bg_focal_loss = (
                0.20
                * (bg_focal * bg_w).sum()
                / (float(max(bg_logits.numel(), 1)) * scale_det)
            )
            support_loss = support_loss + bg_focal_loss

            if dist_norm is not None:
                dist_bias = head.compute_prompt_bias(dist_norm)
                max_fg_cos = ((vis_norm[bg_idx] @ txt_norm.t()) - fg_bias).max(dim=-1).values
                max_dist_cos = ((vis_norm[bg_idx] @ dist_norm.t()) - dist_bias).max(dim=-1).values
                bg_dist_loss = 0.04 * F.relu(max_fg_cos - max_dist_cos).mean()
                support_loss = support_loss + bg_dist_loss

        kd_and_nce = self.kl_weight * kl_loss + self.infonce_weight * infonce_loss
        total = support_loss + kd_and_nce + cosine_loss
        return OpenVocabLossOutput(
            total=total,
            focal=support_loss,
            teacher_kl=kd_and_nce,
            teacher_cosine=cosine_loss,
            num_foreground=num_fg,
            num_background=num_bg,
            num_oov=num_oov,
        )


VALID_CONTAINER_PARTS: dict[str, frozenset[str]] = {
    "building": frozenset(
        {
            "window",
            "door",
            "signboard",
            "pillar",
            "stairs",
            "canopy",
            "street_light",
            "traffic_light",
            "traffic_sign",
        }
    ),
    "canopy": frozenset(
        {
            "window",
            "door",
            "signboard",
            "pillar",
            "stairs",
            "canopy",
            "street_light",
            "traffic_light",
            "traffic_sign",
        }
    ),
    "car": frozenset({"license_plate", "window"}),
    "truck": frozenset({"license_plate", "window"}),
    "bus": frozenset({"license_plate", "window"}),
    "wheeled_transport": frozenset({"license_plate", "window"}),
    "table": frozenset({"book", "flowerpot", "cushion"}),
    "cabinet": frozenset({"book", "flowerpot", "cushion"}),
}


def filter_semantic_detections(
    boxes: Tensor,
    obj_scores: Tensor,
    probs: Tensor,
    categories: Sequence[str],
    *,
    quads: Tensor | None = None,
    same_class_iou: float = 0.35,
    same_class_iomin: float = 0.60,
    cross_class_iou: float = 0.55,
) -> Tensor:
    """Deduplicate same-class overlaps and suppress spurious nested sub-part boxes.

    /// @param boxes Axis-aligned bounding boxes [N, 4] (xyxy).
    /// @param obj_scores Class-agnostic detector objectness/quality scores [N].
    /// @param probs Per-class probabilities [N, C].
    /// @param categories Vocabulary category names of length C.
    /// @param quads Optional oriented quadrilaterals [N, 4, 2] for exact polygon overlap.
    /// @param same_class_iou IoU threshold for deduplicating identical classes.
    /// @param same_class_iomin IoMin containment threshold for deduplicating identical classes.
    /// @param cross_class_iou IoU threshold for suppressing cross-class same-box collisions.
    /// @return Long index tensor [K] of kept detection indices in original order.
    """
    n = int(boxes.shape[0])
    if n == 0 or probs.numel() == 0:
        return torch.empty((0,), dtype=torch.long, device=boxes.device)

    top_probs, cls_ids = probs.float().max(dim=-1)
    joint_scores = torch.sqrt(
        (obj_scores.float().clamp(min=0.0) * top_probs.clamp(min=0.0)).clamp(min=0.0)
    )

    boxes_f = boxes.float()
    bw = (boxes_f[:, 2] - boxes_f[:, 0]).clamp(min=0.0)
    bh = (boxes_f[:, 3] - boxes_f[:, 1]).clamp(min=0.0)
    b_ious = box_iou(boxes_f, boxes_f).detach().cpu()
    b_areas = (bw * bh).detach().cpu().clamp(min=1e-6)
    b_min_areas = torch.minimum(b_areas[:, None], b_areas[None, :]).clamp(min=1e-6)
    b_inter = b_ious * (b_areas[:, None] + b_areas[None, :]) / (1.0 + b_ious).clamp(min=1e-6)
    b_iomins = b_inter / b_min_areas

    if quads is not None and quads.shape[0] == n:
        q_ious = pairwise_quad_iou(quads.float(), quads.float()).detach().cpu()
        areas = quad_area(quads.float()).detach().cpu().clamp(min=1e-6)
        min_areas = torch.minimum(areas[:, None], areas[None, :]).clamp(min=1e-6)
        intersections = q_ious * (areas[:, None] + areas[None, :]) / (1.0 + q_ious).clamp(min=1e-6)
        q_iomins = intersections / min_areas
        ious = torch.maximum(q_ious, 0.85 * b_ious)
        iomins = torch.maximum(q_iomins, 0.85 * b_iomins)
    else:
        ious = b_ious
        areas = b_areas
        iomins = b_iomins

    min_side_cpu = torch.minimum(bw, bh).detach().cpu().tolist()
    b_areas_cpu = b_areas.tolist()
    cls_cpu = cls_ids.detach().cpu().tolist()
    areas_cpu = areas.tolist()
    obj_cpu = obj_scores.float().detach().cpu().tolist()
    top_p_cpu = top_probs.detach().cpu().tolist()
    order = torch.argsort(joint_scores.detach().cpu(), descending=True, stable=True).tolist()

    suppressed = [False] * n
    # Pass 0: Suppress tiny sub-part fragments contained inside a higher-objectness container box.
    for m in range(n):
        cat_m = categories[cls_cpu[m]] if cls_cpu[m] < len(categories) else ""
        allowed = VALID_CONTAINER_PARTS.get(cat_m)
        area_m = areas_cpu[m]
        for k in range(n):
            if m == k:
                continue
            if (
                areas_cpu[k] < 0.45 * area_m
                and float(iomins[m, k].item()) > 0.75
                and obj_cpu[m] >= obj_cpu[k]
            ):
                cat_k = categories[cls_cpu[k]] if cls_cpu[k] < len(categories) else ""
                if allowed is None or cat_k not in allowed:
                    suppressed[k] = True

    keep_mask = [False] * n

    for idx_pos, i in enumerate(order):
        if suppressed[i]:
            continue
        if (min_side_cpu[i] < 4.5 or b_areas_cpu[i] < 24.0) and top_p_cpu[i] < 0.45:
            suppressed[i] = True
            continue
        keep_mask[i] = True
        cls_i = cls_cpu[i]
        cat_i = categories[cls_i] if cls_i < len(categories) else ""
        allowed_parts = VALID_CONTAINER_PARTS.get(cat_i)
        area_i = areas_cpu[i]
        iou_row = ious[i]
        iomin_row = iomins[i]

        for j in order[idx_pos + 1 :]:
            if suppressed[j]:
                continue
            iou_ij = float(iou_row[j].item())
            iomin_ij = float(iomin_row[j].item())
            cls_j = cls_cpu[j]
            if cls_j == cls_i:
                if iou_ij > same_class_iou or iomin_ij > same_class_iomin:
                    suppressed[j] = True
            else:
                if iou_ij > cross_class_iou:
                    suppressed[j] = True
                elif areas_cpu[j] < 0.45 * area_i and iomin_ij > 0.78:
                    cat_j = categories[cls_j] if cls_j < len(categories) else ""
                    if allowed_parts is None or cat_j not in allowed_parts:
                        suppressed[j] = True

    kept = [idx for idx, flag in enumerate(keep_mask) if flag]
    return torch.tensor(kept, dtype=torch.long, device=boxes.device)


__all__ = [
    "DenseAnchorSemanticHead",
    "OpenVocabLossOutput",
    "OpenVocabularySigmoidLoss",
    "RoIPooledSemanticHead",
    "VALID_CONTAINER_PARTS",
    "filter_semantic_detections",
]
