"""Lightweight class-agnostic student detector for embedded deployment."""

from .assigner import ATSSAssigner, Assignment
from .decoder import Detection, InferenceDecoder
from .head import QuadDetectorOutput
from .model import DetectorOutput, QuadProposalDetector, StudentDetector
from .losses import LossOutput, ProposalLoss
from .quad_decoder import QuadDetection, QuadInferenceDecoder, decode_dense_quad_output
from .targets import TargetBuilder, TrainingTargets

__all__ = [
    "ATSSAssigner",
    "Assignment",
    "Detection",
    "DetectorOutput",
    "InferenceDecoder",
    "LossOutput",
    "QuadDetection",
    "QuadDetectorOutput",
    "QuadInferenceDecoder",
    "QuadProposalDetector",
    "ProposalLoss",
    "StudentDetector",
    "TargetBuilder",
    "TrainingTargets",
    "decode_dense_quad_output",
]
