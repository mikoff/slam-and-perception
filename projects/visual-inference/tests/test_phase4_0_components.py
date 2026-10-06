"""Unit tests for Phase 4.0 ontology, crop extractor, SigLIP teacher, and semantic data."""

from __future__ import annotations

from PIL import Image
import torch

from student_detector.crop_extractor import (
    CropPolicy,
    extract_object_crop,
)
from student_detector.ontology import (
    is_compatible,
    map_to_canonical,
)
from student_detector.semantic_evaluation_data import (
    SemanticGroundTruthRecord,
    _compute_size_bin,
)
from student_detector.siglip import (
    SIGLIP_EMBEDDING_DIM,
    SIGLIP_INPUT_SIZE,
    SiglipTeacher,
)


def test_ontology_mappings() -> None:
    assert map_to_canonical("car") == "car"
    assert map_to_canonical("adult") == "pedestrian"
    assert map_to_canonical("person") == "pedestrian"
    assert map_to_canonical("bicycle") == "cyclist"
    assert map_to_canonical("trafficcone") == "traffic_cone"
    assert map_to_canonical("stroller") == "stroller"
    assert map_to_canonical("dog") == "dog"
    assert map_to_canonical("barrier") == "construction_barrier"
    assert map_to_canonical("explicit_stuff") == "road_surface"

    assert is_compatible("person", "pedestrian")
    assert is_compatible("pedestrian", "pedestrian")
    assert is_compatible("bicycle", "cyclist")
    assert not is_compatible("dog", "car")


def test_crop_extractor_axis_aligned() -> None:
    img = Image.new("RGB", (100, 100), color=(100, 150, 200))
    policy = CropPolicy(mode="axis_aligned", context_margin=0.1, target_size=(224, 224))
    quad = [[10.0, 10.0], [30.0, 10.0], [30.0, 40.0], [10.0, 40.0]]
    crop = extract_object_crop(img, quad, policy)

    assert crop.is_valid
    assert crop.image.size == (224, 224)
    assert crop.crop_mode == "axis_aligned"
    assert crop.bounding_box[0] <= 10.0
    assert crop.bounding_box[1] <= 10.0


def test_crop_extractor_degenerate() -> None:
    img = Image.new("RGB", (100, 100), color=(50, 50, 50))
    policy = CropPolicy(mode="axis_aligned", min_pixel_size=2.0)
    quad = [[10.0, 10.0], [10.5, 10.0], [10.5, 10.5], [10.0, 10.5]]
    crop = extract_object_crop(img, quad, policy)
    assert not crop.is_valid
    assert crop.image.size == (224, 224)


def test_semantic_record_size_bins() -> None:
    assert _compute_size_bin(3.0, 4.0) == "tiny"
    assert _compute_size_bin(20.0, 20.0) == "small"
    assert _compute_size_bin(50.0, 50.0) == "medium"
    assert _compute_size_bin(120.0, 120.0) == "large"

    rec = SemanticGroundTruthRecord(
        annotation_id=1,
        image_id=42,
        source_dataset="coco_2017",
        source_category="dog",
        canonical_category="dog",
        quad=((10.0, 10.0), (30.0, 10.0), (30.0, 30.0), (10.0, 30.0)),
        bbox=(10.0, 10.0, 30.0, 30.0),
        ignore_region=0,
        geometry_tier="source_hbb",
        size_bin="small",
        area=400.0,
    )
    assert rec.is_positive
    assert not rec.is_ignore


def test_siglip_teacher_initialization_and_metadata() -> None:
    teacher = SiglipTeacher(device="cpu")
    meta = teacher.metadata
    assert meta.embedding_dim == SIGLIP_EMBEDDING_DIM == 768
    assert meta.input_size == SIGLIP_INPUT_SIZE == (224, 224)
    assert meta.logit_scale > 0
    assert meta.logit_bias < 0


def test_siglip_text_bank_and_similarity() -> None:
    teacher = SiglipTeacher(device="cpu")
    categories = ["car", "pedestrian", "dog"]
    bank, metadata = teacher.build_text_bank(categories)

    assert bank.shape == (3, 768)
    norms = bank.norm(dim=-1)
    torch.testing.assert_close(norms, torch.ones(3), atol=1e-4, rtol=1e-4)
    assert metadata["categories"] == categories

    dummy_img = Image.new("RGB", (224, 224), color=(255, 0, 0))
    img_embeds = teacher.encode_images([dummy_img])
    assert img_embeds.shape == (1, 768)
    img_norm = img_embeds.norm(dim=-1)
    torch.testing.assert_close(img_norm, torch.ones(1), atol=1e-4, rtol=1e-4)

    logits, probs = teacher.compute_similarity(img_embeds, bank)
    assert logits.shape == (1, 3)
    assert probs.shape == (1, 3)
    assert (probs >= 0.0).all() and (probs <= 1.0).all()
