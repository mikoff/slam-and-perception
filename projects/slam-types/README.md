---
topic: Type-safe SLAM primitives
tags: [slam, type-safety, strong-id, coordinate-frames, geometry, header-only, c++20]
related_nodes: [gtsam, symforce]
---

# slam-types — Type-Safe SLAM Primitives

A **C++20 header-only library** providing two compile-time safety mechanisms for
SLAM and robotics codebases:

| Module | Header | Purpose |
|--------|--------|---------|
| **core** | `slam/core/strong_id.hpp` | Strong-typed identifiers that prevent ID mix-ups |
| **geometry** | `slam/geometry/safe_geometry.hpp` | Frame-tagged transforms that enforce coordinate-frame consistency |

Zero runtime overhead and external dependencies for the core library.

---

## Motivation

### The ID Problem

In a SLAM system, `PoseId(42)` and `LandmarkId(42)` are semantically different
entities sharing the same raw integer. Passing one where the other is expected
compiles silently and produces **wrong results**. `StrongId` eliminates this
class of bugs entirely:

```cpp
PoseId p(42);
LandmarkId l(42);
// p == l;  // COMPILE ERROR — different types
```

### The Frame Problem

Matrix multiplication is the single most error-prone operation in a geometry
codebase. Given `T_map_cam` and `T_veh_sensor`, the expression
`T_map_cam * T_veh_sensor` **compiles and runs**, yet the math is nonsensical
because the adjacent frames don't cancel. Safe geometry catches this at compile
time with a `static_assert`.

---

## Core Concepts

### 1. StrongId — Strong-Typed Identifiers

`StrongId<Tag, T>` wraps an integral value with a compile-time **tag**
type, making IDs from different domains incompatible:

```cpp
#include <slam/core/strong_id.hpp>

struct PoseTag {};
struct LandmarkTag {};

using PoseId     = slam::core::StrongId<PoseTag>;
using LandmarkId = slam::core::StrongId<LandmarkTag>;

PoseId p1(42);
LandmarkId l1(42);

p1.isValid();          // true
PoseId().isValid();    // false -- default-constructed = invalid

// Works out-of-the-box with STL containers:
std::unordered_map<PoseId, Pose3d> graph;
std::map<LandmarkId, Point3d> landmarks;
```

**Design rules:**
- Call `.value()` **only** at system boundaries (serialization, logging, C APIs).
- Never do arithmetic on IDs.
- Use `PoseId::Invalid()` instead of magic numbers like `-1`.

### 2. Safe Geometry -- Frame-Tagged Transforms

Spatial states and spatial actions are mathematically identical (Lie group
duality). A **Pose** of entity V in reference frame M *is* the **Transform**
that maps points from V to M:

$$T_{M \leftarrow V}$$

```cpp
#include <slam/geometry/safe_geometry.hpp>

struct MapFrame {};
struct VehicleFrame {};
struct SensorFrame {};

// Wraps any math backend (gtsam::Pose2, sym::Pose2d, Eigen, etc.)
using PoseVehicleInMap    = slam::geometry::Pose<gtsam::Pose2, VehicleFrame, MapFrame>;
using ToVehicleFromSensor = slam::geometry::Transform<gtsam::Pose2, VehicleFrame, SensorFrame>;
using PointInVehicle      = slam::geometry::TaggedPoint<gtsam::Point2, VehicleFrame>;
```

**Composition — compile-time frame cancellation:**

```cpp
PoseVehicleInMap    T_map_veh(gtsam::Pose2(5.0, 0.0, 0.0));
ToVehicleFromSensor T_veh_sens(gtsam::Pose2(1.0, 0.0, 0.0));

auto T_map_sens = T_map_veh * T_veh_sens;  // OK: Veh cancels
// auto bad = T_map_veh * T_map_veh;        // COMPILE ERROR: Map != Veh
```

**Point projection:**

```cpp
auto p_map = T_map_veh * p_veh;  // OK: frames match
// auto bad = T_map_veh * p_map;  // COMPILE ERROR: Veh != Map
```

**Between:**

```cpp
auto T_a_b = slam::geometry::between(pose_a, pose_b);
```

### 3. The Math Behind It

#### The "Attached Frame" Mental Model
To visualize coordinate frame math, think in terms of rigidly attached frames. In the beginning, all frames are perfectly aligned at the origin.
- The Pose (The Move): To move the vehicle to its current state, you take the vehicle frame (while aligned with the map), rotate it in place, and then translate (pin) it to a new point on the map.
- The Reprojection (The Action): When you have a sensor reading $p_{veh}$, you resolve it globally in two steps:
  - Virtual Alignment: Project the local point into "virtual axes" centered at the vehicle's origin, but aligned with the map's direction. (Mathematically, this is $R \cdot p_{veh}$).
  - The Shift: Add the translation vector of the vehicle's origin with respect to the map origin. (Mathematically, this is $+ t$).

#### The Golden Rule: Rotate THEN Translate

$$p_{map} = R \cdot p_{veh} + t$$

Rotation spins the point around the frame's own origin (safe in-place).
Translation then pins the result into the target frame. Reversing this order
swings the frame in an arc around the wrong origin.

#### Chaining: Right-to-Left Frame Cancellation

$$T_{map \leftarrow sensor} = T_{map \leftarrow veh} \cdot T_{veh \leftarrow sensor}$$

Adjacent `veh` tags touch and cancel, leaving only `map ← sensor`.
The `operator*` in this library enforces this rule with `static_assert`.

#### Interactive Visualization

Open `frame_visualization.html` in a browser to interactively explore the
Pose → Rotate → Translate → Reproject pipeline with adjustable sliders.

### 4. GeometryTraits -- Backend Adaptation

The default traits assume the math backend provides `inverse()` and
`operator*`. For backends with a different API, specialize `GeometryTraits`:

```cpp
// Default (works with gtsam::Pose2, gtsam::Pose3):
template <typename T>
struct GeometryTraits {
    static T inverse(const T& v) { return v.inverse(); }
    static T compose(const T& a, const T& b) { return a * b; }
    template <typename P>
    static P action(const T& pose, const P& pt) { return pose * pt; }
};
```

For SymForce (uses `Inverse()` instead of `inverse()`):

```cpp
#include <slam/geometry/traits/symforce_traits.hpp>
// Specialization for sym::Pose2<Scalar> is automatically registered.
```

### 5. Keeping Geometry Pure

A `Transform<Backend, A, B>` knows **nothing** about identity or time.
When you need to associate an ID or timestamp with a pose, use a domain struct:

```cpp
struct TrackedPoseNode {
    PoseId id;
    uint64_t timestamp_ns;
    Pose2 T_map_vehicle;
};

struct VisualObservation {
    PoseId observer_pose_id;
    CameraId camera_id;
    LandmarkId landmark_id;
    uint64_t timestamp_ns;
    Eigen::Vector2d pixel_coords;
};
```

This separation keeps mathematical operator overloading clean and prevents
logical contradictions.

### 6. How to Store Them, two options
Simply store the TrackedPose domain struct in your map.
```c++
class SlamState {
    std::unordered_map<PoseId, TrackedPose> poses;

public:
    Pose getPose(PoseId id) { return poses.at(id).transform_to_map; }
    uint64_t getTime(PoseId id) { return poses.at(id).timestamp_ns; }
};
```
- **Pros**: Very easy to reason about. Everything about a state is bundled together.
- **Cons**: When a backend optimizer runs, it iterates over thousands of poses, doing heavy matrix multiplication. It doesn't care about timestamps. Storing them interleaved in memory with the matrices can cause CPU cache misses.

Separate the geometry from the temporal data in your core state registry.
```c++
class SlamState {
    // For the Math Optimizer (Dense, fast memory access)
    std::unordered_map<PoseId, Pose> poses; 
    
    // For the Frontend / Interpolator
    std::unordered_map<PoseId, uint64_t> timestamps; 
    // Or even an ordered map if you need to query by time frequently:
    // std::map<uint64_t, PoseId> time_to_pose;
};
```
-- **Pros**: The math optimizer only touches the poses map, keeping the CPU cache full of pure geometry. This is how highly optimized libraries are structured under the hood.

---

## Integration

### Consuming from another CMake project in this monorepo

```cmake
# In your CMakeLists.txt:
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../slam-types
    ${CMAKE_CURRENT_BINARY_DIR}/slam-types
    EXCLUDE_FROM_ALL
)
target_link_libraries(my_target PRIVATE slam_types)
```

Then include headers with the full path:

```cpp
#include <slam/core/strong_id.hpp>
#include <slam/geometry/safe_geometry.hpp>
```

---

## Build

### Core tests only (no backend dependencies)

```bash
cmake -B build -DSLAM_TYPES_TEST_GTSAM=OFF -DSLAM_TYPES_TEST_SYMFORCE=OFF
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

### With GTSAM integration tests

```bash
cmake -B build -DSLAM_TYPES_TEST_SYMFORCE=OFF
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

### Full test suite (GTSAM + SymForce)

```bash
cmake -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

> GTSAM 4.3a0 must be installed on the system (the Docker image includes it).
> SymForce v0.10.1 is fetched automatically by CMake via FetchContent.

---

## Project Layout

```
slam-types/
├── CMakeLists.txt                  # INTERFACE library + test targets
├── Dockerfile                      # GTSAM pre-installed, libgmp for symforce
├── pyproject.toml                  # Minimal Python env
├── README.md                       # This file
├── frame_visualization.html        # Interactive geometry visualization
├── include/
│   └── slam/
│       ├── core/
│       │   └── strong_id.hpp       # Phantom-typed StrongId<Tag, T>
│       └── geometry/
│           ├── safe_geometry.hpp    # Transform, Pose, TaggedPoint, operators
│           └── traits/
│               └── symforce_traits.hpp  # GeometryTraits for sym::Pose2
└── tests/
    ├── test_strong_id.cpp          # StrongId unit tests
    ├── test_safe_geometry.cpp      # Geometry unit tests (mock backend)
    ├── test_gtsam_integration.cpp  # GTSAM backend integration
    └── test_symforce_integration.cpp  # SymForce backend integration
```
