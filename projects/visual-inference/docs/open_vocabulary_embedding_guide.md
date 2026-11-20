# Open-Vocabulary Semantic Embeddings & Edge Vocabulary Guide

This document explains how open-vocabulary semantic embeddings are structured in the MobileNetV4 two-stage pipeline, why the default 37 categories were chosen, and how to scale or customize to arbitrary vocabulary banks (e.g., 128 classes or domain-specific sets) with **zero detector retraining**.

---

## 1. Architectural Philosophy: Decoupled Open-Vocabulary Detection

In conventional closed-set detectors (like YOLO or Faster R-CNN), classification heads output fixed logits for $C$ predefined classes. Adding or modifying a class requires redesigning the head and retraining the entire neural network.

Our pipeline uses a **decoupled representation paradigm**:
1. **Edge Student Detector (`MobileNetV4 + RegionEmbeddingHead`)**:
   - Computes multi-scale feature maps ($P_2$ to $P_5$) and detects class-agnostic bounding geometries (HBBs and Quads).
   - Uses [`MultiScaleRoIAlign`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/region_head.py#L38-L100) to pool $3 \times 3$ feature maps and a lightweight 2-layer MLP (837k parameters, $<0.084$ GMACs) to project each proposal into a **768-dimensional unit L2 visual embedding $\mathbf{z}_{\text{student}}$**.
   - The detector does not know or care what classes exist; it is trained to match SigLIP's continuous multimodal latent space.
2. **Edge Semantic Classifier (`VocabularyBank`)**:
   - A static pre-encoded matrix $W \in \mathbb{R}^{C \times 768}$ containing normalized text embeddings generated offline by SigLIP.
   - Classification at runtime is a single matrix multiplication:
     $$\mathbf{logits} = \mathbf{z}_{\text{student}} W^T \in \mathbb{R}^{N \times C}$$
   - **Zero Vision Transformers and zero text encoders run on the edge hardware.**

---

## 2. Canonical 37-Class Default Ontology

The default 37 classes defined in [`student_detector/ontology.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/ontology.py) were curated to represent autonomous navigation, robotics safety, and background clutter across our 5 datasets:

| Group | Count | Categories | Domain Justification |
|---|---|---|---|
| **Base Traffic & Navigation** | 6 | `car`, `pedestrian`, `cyclist`, `traffic_cone`, `bus`, `truck` | Classical benchmark navigation agents and dynamic obstacles |
| **Vulnerable & Rare Objects** | 5 | `stroller`, `dog`, `debris`, `wheelchair`, `construction_barrier` | Low-frequency critical edge cases flagged during SAM discovery |
| **Street Furniture & Assets** | 14 | `pole`, `signboard`, `street_light`, `bench`, `trash_bin`, `fire_hydrant`, `parking_slot`, `parking_meter`, `canopy`, `cart`, `golf_cart`, `luggage_and_bags`, `pillar`, `license_plate` | Urban infrastructure, hazards, and static obstacles |
| **Architectural Elements** | 7 | `tree`, `window`, `door`, `building`, `motorcycle`, `bicycle`, `stairs` | Structural assets and secondary vehicles from ADE20K & OpenImages |
| **Background Distractors** | 5 | `road_surface`, `sky`, `vegetation`, `building_wall`, `ego_platform_bodywork` | Crucial background absorbers to prevent false positives |

### Why Distractor Classes Are Crucial
In an open-vocabulary system evaluated via Categorical Softmax, if a bounding box lands on road tarmac or concrete and no background categories exist, the probability mass must sum to 1.0, forcing the model to pick an object class (e.g., `debris` or `car`). Including canonical background distractors (`road_surface`, `building_wall`, `sky`) absorbs background noise and allows the pipeline to safely suppress non-objects.

---

## 3. How to Create an Arbitrary Vocabulary (e.g. 128 Classes)

Because the student detector outputs generic 768-D embeddings, **you never need to retrain or fine-tune the detector to add or change classes**.

### Step 1: Generate the Vocabulary Bank Offline (Takes ~1 second)

Run the offline encoding utility using [`student_detector/vocabulary.py`](file:///home/sashamikoff/git/skillup/projects/visual-inference/student_detector/vocabulary.py):

```python
from student_detector.siglip import SiglipTeacher
from student_detector.vocabulary import VocabularyBank

# Define your custom categories (e.g., 128 classes)
custom_categories = [
    # Core obstacles
    "pedestrian", "car", "cyclist", "truck", "bus", "motorcycle",
    # Specific urban robotics objects
    "fire truck", "skateboarder", "electric scooter", "stroller", "wheelchair",
    "dog", "cat", "cardboard box", "pallet", "forklift", "shopping cart",
    "traffic light", "pothole", "manhole cover", "fire hydrant", "backpack",
    "suitcase", "traffic cone", "construction barrier", "bollard",
    # ... any domain-specific strings up to 128 ...
    # Recommended: always include background distractors
    "road_surface", "building_wall", "vegetation", "sky", "vehicle_bodywork"
]

# Instantiate offline teacher (requires GPU or CPU workstation)
teacher = SiglipTeacher(device="cuda")

# Generate text bank (applies prompt ensembles like "a photo of a {category}")
vocab_bank = VocabularyBank.from_teacher(teacher, custom_categories)

# Save compact binary checkpoint
vocab_bank.save("artifacts/vocabulary_bank_128classes.pt")
print(f"Created {vocab_bank.num_classes}-class bank: {vocab_bank.embeddings.shape}")
```

### Memory Footprint Comparison

| Class Count $C$ | Tensor Dimensions | Float32 File Size | Edge RAM | Dot-Product Latency (CPU) |
|---|---|---|---|---|
| **37 classes** (current) | `[37, 768]` | 113.7 KB | $< 0.2$ MB | $< 0.05$ ms |
| **128 classes** | `[128, 768]` | 393.2 KB | $< 0.5$ MB | $< 0.10$ ms |
| **500 classes** | `[500, 768]` | 1.54 MB | $< 2.0$ MB | $< 0.35$ ms |
| **1,000 classes** (LVIS) | `[1000, 768]` | 3.07 MB | $< 4.0$ MB | $< 0.70$ ms |

---

## 4. Edge Runtime Execution (Raspberry Pi 5)

At runtime on the edge device, you load only the lightweight detector and the vocabulary `.pt` file. SigLIP and PyTorch Vision Transformers are completely absent:

```python
import torch
from student_detector.model import StudentDetector
from student_detector.region_head import RegionEmbeddingHead
from student_detector.vocabulary import VocabularyBank

# 1. Load vocabulary bank (393 KB)
vocab = VocabularyBank.load("artifacts/vocabulary_bank_128classes.pt", device="cpu")

# 2. Run detector on camera frame
# MobileNetV4 extracts P2-P5 features; RegionEmbeddingHead outputs [N, 768]
# proposals: [N, 4] bounding boxes, embeddings: [N, 768] unit L2 vectors
with torch.no_grad():
    proposals, embeddings = detector_pipeline(camera_frame)

    # 3. Classify via single matrix multiplication: [N, 768] @ [768, C] -> [N, C]
    logits, probabilities = vocab.classify(embeddings, mode="softmax")
    top_scores, top_class_ids = probabilities.max(dim=-1)

for i in range(len(proposals)):
    class_name = vocab.categories[top_class_ids[i]]
    confidence = top_scores[i].item()
    print(f"Detected {class_name} ({confidence * 100:.1f}%) at {proposals[i].tolist()}")
```

---

## 5. Dynamic Context Switching

Because vocabulary matrices are tiny ($< 1$ MB), edge systems can swap vocabularies dynamically without restarting the vision model:

- **Highway Navigation Mode**: `vocab_highway.pt` (cars, trucks, debris, lanes, barriers)
- **Warehouse Inspection Mode**: `vocab_warehouse.pt` (forklifts, pallets, workers, boxes)
- **Pedestrian Zone Mode**: `vocab_pedestrian.pt` (pedestrians, strollers, dogs, wheelchairs, bikes)

Switching modes is as simple as reassigning the `vocab` reference in memory in $< 1$ ms.
