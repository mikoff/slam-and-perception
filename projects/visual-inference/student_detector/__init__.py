"""Lightweight class-agnostic student detector for embedded deployment."""

from .assigner import ATSSAssigner, Assignment
from .decoder import Detection, InferenceDecoder
from .model import DetectorOutput, StudentDetector
from .losses import LossOutput, ProposalLoss
from .targets import TargetBuilder, TrainingTargets

__all__ = [
    "ATSSAssigner",
    "Assignment",
    "Detection",
    "DetectorOutput",
    "InferenceDecoder",
    "LossOutput",
    "ProposalLoss",
    "StudentDetector",
    "TargetBuilder",
    "TrainingTargets",
]
