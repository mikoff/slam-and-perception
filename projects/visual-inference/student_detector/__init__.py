"""Lightweight class-agnostic student detector for embedded deployment."""

from .assigner import ATSSAssigner, Assignment
from .decoder import Detection, InferenceDecoder
from .model import DetectorOutput, StudentDetector

__all__ = [
    "ATSSAssigner",
    "Assignment",
    "Detection",
    "DetectorOutput",
    "InferenceDecoder",
    "StudentDetector",
]
