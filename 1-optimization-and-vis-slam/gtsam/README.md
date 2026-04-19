---
topic: GTSAM factor graph optimization
tags: [gtsam, slam, factor graph, optimization, pose graph]
scope: Local to gtsam optimization
---

# GTSAM Pose-Graph Optimizer

Uses [GTSAM](https://gtsam.org/) to build and optimize a 2D pose graph with odometry factors and a loop closure.

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
./build/gtsam_example
```

> GTSAM 4.3a0 (C++) is compiled into the Docker image and installed to `/usr/local`.

## Python environment

```bash
uv sync
```

> Installs gtsam (Python wheel), numpy, matplotlib, and ipykernel into `.venv/`.
> The C++ and Python gtsam versions are independent.

## Project layout

```
gtsam/
├── CMakeLists.txt        # find_package(GTSAM) — links against system install
├── Dockerfile            # debian:trixie-slim + GTSAM 4.3a0 from source + uv
├── pyproject.toml        # Python deps: gtsam wheel, numpy, matplotlib, ipykernel
├── build/                # CMake build artefacts (out-of-source)
├── notebook/
│   └── pose_graph.ipynb  # 2D pose-graph example using gtsam Python bindings
└── src/
    └── main.cpp          # 2D pose-graph example (C++)
```
