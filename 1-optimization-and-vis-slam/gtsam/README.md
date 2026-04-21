---
topic: GTSAM factor graph optimization
tags: [gtsam, slam, factor graph, optimization, pose graph, nuscenes, mcap]
scope: Local to gtsam optimization
---

# GTSAM Pose-Graph Optimizer

Uses [GTSAM](https://gtsam.org/) to build and optimize a 2D pose graph with odometry factors and a loop closure, and to load NuScenes sensor data from MCAP files.

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
```

> GTSAM 4.3a0 (C++) is compiled into the Docker image and installed to `/usr/local`.

### Targets

| Target | Description |
|---|---|
| `gtsam_example` | 2D pose-graph optimisation demo |
| `nuscenes_loader` | Loads NuScenes MCAP sensor data |

## NuScenes MCAP loader

Reads MCAP files from `data/nuscenes/mcap/` and decodes all sensor streams:

```bash
./build/nuscenes_loader ../../data/nuscenes/mcap/NuScenes-v1.0-mini-scene-0061.mcap
```

### Supported topics

| Topic | Encoding | Domain type |
|---|---|---|
| `/imu` | JSON | `ImuMeasurement` — `TaggedPoint<Vector3, ImuFrame>` |
| `/odom` | JSON | `OdomMeasurement` — `Pose<Pose3, BodyFrame, MapFrame>` |
| `/gps` | protobuf `foxglove.LocationFix` | `GnssFix` — `TaggedPoint<Point3, Wgs84Frame>` |
| `/LIDAR_TOP` | protobuf `foxglove.PointCloud` | `Stamped<foxglove::PointCloud>` |
| `/RADAR_*` | protobuf `foxglove.PointCloud` | `Stamped<foxglove::PointCloud>` |
| `/CAM_*/image_rect_compressed` | protobuf `foxglove.CompressedImage` | `Stamped<foxglove::CompressedImage>` |
| `/tf` | protobuf `foxglove.FrameTransform` | `Stamped<foxglove::FrameTransform>` |

IMU, odometry, and GNSS are wrapped in frame-tagged types from `slam-types`.
Point clouds, images, and TF stay as raw protobuf (runtime frame IDs).

### Dependencies (via FetchContent)

- [foxglove/mcap](https://github.com/foxglove/mcap) v2.1.3 — MCAP C++ reader
- [nlohmann/json](https://github.com/nlohmann/json) v3.11.3 — JSON parsing

### Dependencies (system, installed in Docker image)

- `protobuf-compiler` + `libprotobuf-dev` — protobuf code generation and runtime
- `liblz4-dev`, `libzstd-dev` — MCAP compression support
- `python3` + `mcap` + `protobuf` Python packages — schema extraction at build time

### Protobuf schema generation

Schemas are **automatically extracted** from the MCAP data files at build time.
The build reads embedded `FileDescriptorSet` objects from one MCAP file, writes
`.proto` files, and runs `protoc` — all inside `build/`. Nothing is committed.

Prerequisite: MCAP data must exist at `data/nuscenes/mcap/` (run `scripts/download_data.py`).

## Python environment

```bash
uv sync
```

> Installs gtsam (Python wheel), numpy, matplotlib, and ipykernel into `.venv/`.

## Project layout

```
gtsam/
├── CMakeLists.txt                     # FetchContent(mcap, json), auto proto generation
├── Dockerfile                         # debian:trixie-slim + GTSAM + protobuf + lz4 + zstd
├── pyproject.toml
├── scripts/
│   ├── download_data.py               # Downloads nuScenes-mini and converts to MCAP
│   └── extract_and_compile_protos.py  # Extracts .proto from MCAP, runs protoc
├── build/                             # CMake build artefacts (all generated code lives here)
│   ├── schemas/                       #   .proto extracted at build time
│   └── proto-gen/                     #   .pb.h / .pb.cc compiled at build time
├── notebook/
│   └── pose_graph.ipynb
└── src/
    ├── main.cpp                       # 2D pose-graph example (gtsam_example)
    ├── domain_types.hpp               # Frame tags for pose-graph example
    ├── pose_graph.hpp                 # PoseGraphBuilder wrapper
    ├── nuscenes_main.cpp              # NuScenes MCAP loader entry point
    ├── nuscenes_mcap_loader.hpp/.cpp  # MCAP reader + decoder
    └── nuscenes_domain_types.hpp      # Frame-tagged types for NuScenes data
```
